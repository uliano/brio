/*
 * usart.hpp
 *
 * The AVR DA/DB USART (DS40002247B ch. 27) in two strata, as
 * docs/avrdx/usart.md describes:
 *
 *  RESOURCE - Usart<n>: the typed view of one instance. A config struct
 *  owns the whole configuration (route, communication mode, frame
 *  format, receiver mode, baud register, the half-duplex bits, the
 *  IRCOM pulse lengths); init<cfg>() folds it and refuses at compile
 *  time what this package cannot bond, init(cfg) computes it at run
 *  time and returns false instead. Then the verbs the modes share:
 *  enable/disable of each direction, the frame format, the baud
 *  register and its readback, the status flags with their write-one-
 *  to-clear discipline, the interrupt enables, the ISR bodies
 *  (receive / clear_txc), and release() - route back to NONE with the
 *  pins handed to PORT.
 *
 *  TASKS - what an application names, each a mode with its knobs in
 *  the application's units (a clock and a baud rate; the resource
 *  speaks BAUD register values):
 *    Uart<n, route>        interrupt-driven full-duplex byte transport
 *                          (ByteSink + ByteSource; the console)
 *    OneWire<n, route>     half duplex on TXD (LBME + ODME + pull-up)
 *    Rs485<n, route>       transport with the automatic XDIR drive enable
 *    SyncHost<n, route>    synchronous USART, XCK driven out
 *    SyncClient<n, route>  synchronous USART, XCK taken in
 *    MspiHost<n, route>    Host SPI mode (UDORD/UCPHA, S = 2)
 *    IrdaLink<n, route>    IRCOM with the TX/RX pulse-length knobs
 *    AutoBaud<n, route>    GENAUTO / LINAUTO arming and recovery
 *  A task owns its instance; two tasks on one Usart<n> is the app's bug.
 *  MPCM is not a task: it is a receiver filter on the resource
 *  (multiprocessor(bool)), and start-of-frame detection is a verb, not
 *  a configuration field - see the errata below.
 *
 * Facts that shape the code (27.3, 27.5, errata DS80000915F 2.16.x and
 * DS80000882C 2.15.x):
 *  - the fractional baud generator wants the divisor left-shifted by
 *    six: BAUD = 64 x f_CLK_PER / (S x f_baud) with S = 16 (normal),
 *    8 (double speed) or 2 (synchronous and Host SPI); the register's
 *    valid range is 64..65535, and in the synchronous modes only
 *    BAUD[15:6] counts - the six low bits must be zero;
 *  - RXDATAH carries the status of the frame at the head of the RX
 *    FIFO and the read of ONE of the two data registers shifts the
 *    FIFO: RXDATAH first, then RXDATAL - except in 9-bit low-byte-
 *    first (9BITL), where RXDATAH is the shifting read and RXDATAL
 *    must be read first. The transmit side mirrors it: TXDATAL before
 *    TXDATAH in 9BITL, TXDATAH before TXDATAL in 9BITH;
 *  - STATUS is a mix: RXCIF/DREIF are read-only conditions, while
 *    TXCIF/RXSIF/ISFIF/BDF are write-one-to-clear - cleared with a
 *    PLAIN store of just that bit (an RMW would read every pending
 *    flag back as one and clear the lot), the pin.hpp discipline;
 *  - the RX buffer is two frames deep plus the shift register; BUFOVF
 *    is set when a new start bit arrives with all three full, and it
 *    travels with its frame through the FIFO;
 *  - disabling the transmitter waits for what is in flight and then
 *    hands TXD back to PORT as an INPUT; disabling the receiver is
 *    immediate and flushes the buffer;
 *  - errata 2.16.1 / 2.15.1 (open drain): with the TXD pin configured
 *    as an output the pin drives high whether ODME is set or not. The
 *    documented work around - and what init() does for a one-wire
 *    config - is to leave TXD's direction at INPUT and let the pull-up
 *    hold the line, which contradicts 27.3.1's "the TXD pin is
 *    automatically set to output by hardware";
 *  - errata 2.16.2 / 2.15.2 (start-of-frame): SFDEN set while the
 *    device is in Active mode makes a read of RXDATA re-trigger the
 *    detector and corrupt the frame being received. SFDEN is therefore
 *    not a configuration field here but a pair of verbs,
 *    arm_start_of_frame() / disarm_start_of_frame(), to be used around
 *    the transition to standby;
 *  - errata 2.16.3 (auto-baud): once ISFIF is set the receiver is dead
 *    and clearing the flag does not revive it - RXEN must be written 0
 *    then 1. Every auto-baud path here goes through recover_from_isf();
 *  - the routes are PORTMUX.USARTROUTEA/B, two bits per instance, with
 *    a NONE code that disconnects every pin: an instance still runs
 *    pinless (the baud generator, the transmitter's flags and its
 *    timing are all live) but LBME and ODME are not available there -
 *    both act on the TXD PAD, and a pinless loop-back receives nothing
 *    (bench). Which positions a package bonds is the device header's
 *    route enums - the table below is those enums, package by package.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <avr/io.h>

#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform_avr.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

// ---- routes and pins (17.5.9 - 17.5.10) --------------------------------------

/// USART pin routing (PORTMUX.USARTROUTEA/B). The values ARE the
/// device header's own group values, `none` included: an instance with
/// no pins still runs (baud generator, internal loop-back).
enum class UsartRoute : uint8_t {
    def = 0,
    alt1 = 1,
    none = 3,
};
static_assert(static_cast<uint8_t>(UsartRoute::def) == PORTMUX_USART0_DEFAULT_gv &&
              static_cast<uint8_t>(UsartRoute::alt1) == PORTMUX_USART0_ALT1_gv &&
              static_cast<uint8_t>(UsartRoute::none) == PORTMUX_USART0_NONE_gv,
              "the route codes must be the device header's group values");

/// The name the console call sites use.
using Route = UsartRoute;

/// The pin count of this package, read off the device header's own
/// tiers (PORTG and USART5 are 64-pin, USART3 is 48-pin and up, TWI1
/// is 32-pin and up). It is what distinguishes the four route tables
/// below; nothing else in brio needs it, so it lives here.
#if defined(USART5)
inline constexpr uint8_t usart_package_pins = 64;
#elif defined(USART3)
inline constexpr uint8_t usart_package_pins = 48;
#elif defined(TWI1)
inline constexpr uint8_t usart_package_pins = 32;
#else
inline constexpr uint8_t usart_package_pins = 28;
#endif

/// The port every USART's four signals live on: one port per instance,
/// the default route on pins 0..3 and ALT1 on pins 4..7 of it.
constexpr char usart_port_letter(uint8_t n) {
    switch (n) {
        case 0: return 'A';
        case 1: return 'C';
        case 2: return 'F';
        case 3: return 'B';
        case 4: return 'E';
        case 5: return 'G';
        default: return '?';
    }
}

/// The four signals of a route (27.2.2), in pin order.
enum class UsartSignal : uint8_t { txd = 0, rxd = 1, xck = 2, xdir = 3 };

/// Where one signal of one route sits, and whether THIS package bonds
/// it out. `bonded == false` means the silicon has the function but no
/// pin for it here: the instance stays usable, only the modes that
/// need that signal are refused.
struct UsartPin {
    char port = '?';
    uint8_t pin = 0;
    bool bonded = false;
    constexpr explicit operator bool() const { return bonded; }
};

/// Whether this package offers a route at all (the device header's
/// route enum for the instance either lists ALT1 or it does not).
constexpr bool usart_route_exists(uint8_t n, UsartRoute r) {
    if (n >= usart_count) return false;
    if (r == UsartRoute::none || r == UsartRoute::def) return true;
    switch (n) {
        case 0: return true;                              // PA4..PA7 on every package
        case 1: return usart_package_pins >= 48;          // PC4..PC7: 48 and 64 pins
        case 2: return usart_package_pins >= 32;          // PF4/PF5: 32 pins and up
        case 3: return true;                              // PB4/PB5: the instance is 48+ already
        case 4: return usart_package_pins >= 64;          // PE4..PE7: 64 pins only
        default: return true;                             // USART5 is 64-pin only
    }
}

/// The position of one signal, package by package. The facts are the
/// device headers' route enum comments:
///   USART0 PA0-PA3 / PA4-PA7        every package, all four signals
///   USART1 PC0-PC3 / PC4-PC7        ALT1 from 48 pins up
///   USART2 PF0-PF3 / PF4,PF5        28 pins bond PF0/PF1 only, so the
///                                   default route has no XCK/XDIR
///                                   there; ALT1 has no XCK position on
///                                   any package and keeps XDIR on PF3
///                                   (AVR DB only - the DA route tables
///                                   list no XDIR for it)
///   USART3 PB0-PB3 / PB4-PB7        48 pins stop at PB5: ALT1 has
///                                   TXD/RXD but no XCK/XDIR
///   USART4 PE0-PE3 / PE4-PE7        48 pins stop at PE3: no ALT1
///   USART5 PG0-PG3 / PG4-PG7        64 pins only, all four signals
constexpr UsartPin usart_pin(uint8_t n, UsartRoute r, UsartSignal s) {
    if (!usart_route_exists(n, r) || r == UsartRoute::none) {
        return {};
    }
    const char port = usart_port_letter(n);
    const uint8_t idx = static_cast<uint8_t>(s);
    const uint8_t pin = static_cast<uint8_t>((r == UsartRoute::alt1 ? 4 : 0) + idx);
    if (r == UsartRoute::def) {
        // PORTF on a 28-pin package bonds PF0, PF1 and PF6 only.
        if (n == 2 && idx >= 2 && usart_package_pins < 32) return {port, pin, false};
        return {port, pin, true};
    }
    // ALT1
    if (n == 2) {
        if (idx == 2) return {port, pin, false};       // no XCK position in this route
        if (idx == 3) {
#ifdef MVIO
            // AVR DB: the route table keeps XDIR on PF3, the DEFAULT
            // route's own pin.
            return {port, 3, true};
#else
            // AVR DA: no XDIR in this route.
            return {port, pin, false};
#endif
        }
        return {port, pin, true};
    }
    if (n == 3 && idx >= 2 && usart_package_pins < 64) return {port, pin, false};
    return {port, pin, true};
}

// ---- the knobs (27.5.6 - 27.5.11) --------------------------------------------

/// CMODE: which protocol the instance speaks (the values ARE the codes).
enum class UsartMode : uint8_t {
    async = USART_CMODE_ASYNCHRONOUS_gc,  ///< asynchronous USART
    sync = USART_CMODE_SYNCHRONOUS_gc,    ///< synchronous USART on XCK
    ircom = USART_CMODE_IRCOM_gc,         ///< IrDA pulse modulation
    mspi = USART_CMODE_MSPI_gc,           ///< Host SPI
};

/// PMODE.
enum class UsartParity : uint8_t {
    none = USART_PMODE_DISABLED_gc,
    even = USART_PMODE_EVEN_gc,
    odd = USART_PMODE_ODD_gc,
};

/// CHSIZE. The two nine-bit codes differ only in which data register
/// shifts the FIFO - see the read/write order in the header comment.
enum class UsartBits : uint8_t {
    five = USART_CHSIZE_5BIT_gc,
    six = USART_CHSIZE_6BIT_gc,
    seven = USART_CHSIZE_7BIT_gc,
    eight = USART_CHSIZE_8BIT_gc,
    nine_low_first = USART_CHSIZE_9BITL_gc,
    nine_high_first = USART_CHSIZE_9BITH_gc,
};

/// True when the code is one of the two nine-bit ones.
constexpr bool usart_is_9bit(UsartBits b) {
    return b == UsartBits::nine_low_first || b == UsartBits::nine_high_first;
}

/// How many data bits a code carries (5..9).
constexpr uint8_t usart_data_bits(UsartBits b) {
    switch (b) {
        case UsartBits::five: return 5;
        case UsartBits::six: return 6;
        case UsartBits::seven: return 7;
        case UsartBits::eight: return 8;
        default: return 9;
    }
}

/// RXMODE: the receiver's sampling regime. The three non-normal ones
/// are asynchronous-only (27.5.7).
enum class UsartRxMode : uint8_t {
    normal = USART_RXMODE_NORMAL_gc,
    clk2x = USART_RXMODE_CLK2X_gc,      ///< double speed: 8 samples per bit
    genauto = USART_RXMODE_GENAUTO_gc,  ///< generic auto-baud
    linauto = USART_RXMODE_LINAUTO_gc,  ///< LIN constrained auto-baud
};

/// CTRLD.ABW: how far the LIN sync field may drift (27.5.11).
enum class UsartAbWindow : uint8_t {
    wdw0 = USART_ABW_WDW0_gc,   ///< 32 +-6 (18 %)
    wdw1 = USART_ABW_WDW1_gc,   ///< 32 +-5 (15 %)
    wdw2 = USART_ABW_WDW2_gc,   ///< 32 +-7 (21 %)
    wdw3 = USART_ABW_WDW3_gc,   ///< 32 +-8 (25 %)
};

/// The frame format the tasks pass around (CTRLC's normal-mode half).
struct UsartFormat {
    UsartBits bits = UsartBits::eight;
    UsartParity parity = UsartParity::none;
    bool two_stop = false;
};

/// Everything one instance is configured with. `baud` is the BAUD
/// REGISTER value (usart_baud_reg() computes it from a clock and a
/// rate) - the resource speaks the register, the tasks speak hertz.
struct UsartConfig {
    UsartRoute route = UsartRoute::def;
    UsartMode mode = UsartMode::async;
    UsartBits bits = UsartBits::eight;
    UsartParity parity = UsartParity::none;
    bool two_stop = false;              ///< SBMODE (the receiver ignores it)
    UsartRxMode rx_mode = UsartRxMode::normal;
    uint16_t baud = 0;                  ///< BAUD register value, 64..65535
    bool rx = true;                     ///< RXEN
    bool tx = true;                     ///< TXEN
    bool loop_back = false;             ///< LBME: TXD feeds the receiver, RXD is released
    bool open_drain = false;            ///< ODME: the transmitter can only pull low
    bool rs485 = false;                 ///< XDIR driven around every frame
    bool multiprocessor = false;        ///< MPCM: only address frames pass the filter
    bool sync_client = false;           ///< synchronous mode: XCK is an input
    bool lsb_first = false;             ///< Host SPI UDORD
    bool sample_trailing = false;       ///< Host SPI UCPHA
    bool irda_event_input = false;      ///< EVCTRL.IREI: the IRCOM receiver listens to a channel
    uint8_t tx_pulse = 0;               ///< TXPLCTRL (IRCOM): 0 = 3/16 of a bit
    uint8_t rx_pulse = 0;               ///< RXPLCTRL (IRCOM): 0 = no filtering
    UsartAbWindow ab_window = UsartAbWindow::wdw0;
    bool debug_run = false;             ///< keep running while the debugger holds the CPU
};

/// Samples per bit, S in the baud equations (27.3.2.2.1).
constexpr uint8_t usart_samples(UsartMode mode, UsartRxMode rx = UsartRxMode::normal) {
    if (mode == UsartMode::sync || mode == UsartMode::mspi) return 2;
    return rx == UsartRxMode::clk2x ? 8 : 16;
}

/// BAUD = 64 x f_CLK_PER / (S x f_baud), rounded to nearest. In the
/// synchronous modes only BAUD[15:6] is the divisor and the fraction
/// must be zero, so the divisor is rounded and then shifted. Returns 0
/// when the rate is out of the register's 64..65535 range - callers
/// refuse. The intermediate 64/S x f_CLK_PER stays inside 32 bits for
/// peripheral clocks up to 134 MHz (S = 2), so no 64-bit division is
/// dragged into a driver.
constexpr uint16_t usart_baud_reg(uint32_t clk_per_hz, uint32_t baud, uint8_t samples = 16) {
    if (baud == 0 || samples == 0) return 0;
    if (samples == 2) {
        const uint32_t div = (clk_per_hz + baud) / (2u * baud);   // round to nearest
        if (div < 1u || div > 0x3FFu) return 0;
        return static_cast<uint16_t>(div << 6);
    }
    const uint32_t mul = 64u / samples;                            // 4 or 8
    const uint32_t reg = (clk_per_hz * mul + baud / 2u) / baud;
    if (reg < 64u || reg > 65535u) return 0;
    return static_cast<uint16_t>(reg);
}

/// The rate a BAUD register value actually produces at a peripheral
/// clock - what the fractional generator settled on, not what was
/// asked for.
constexpr uint32_t usart_actual_baud(uint32_t clk_per_hz, uint16_t reg, uint8_t samples = 16) {
    if (reg == 0 || samples == 0) return 0;
    if (samples == 2) {
        const uint16_t div = static_cast<uint16_t>(reg >> 6);
        return div ? clk_per_hz / (2u * div) : 0;
    }
    return (clk_per_hz * (64u / samples)) / reg;
}

/// The slowest peripheral clock that can still produce `baud`
/// (BAUD >= 64 means f_CLK_PER >= S x f_baud).
constexpr uint32_t usart_min_clock_hz(uint32_t baud, uint8_t samples = 16) {
    return baud * samples;
}

/// Is this configuration legal on this package? Route must exist; the
/// synchronous roles and Host SPI need an XCK pin; RS-485 needs XDIR;
/// the half-duplex bits need a TXD pin; the non-normal receiver modes
/// are asynchronous-only (27.5.7).
template <uint8_t n>
constexpr bool usart_config_valid(const UsartConfig& c) {
    if (n >= usart_count) return false;
    if (!usart_route_exists(n, c.route)) return false;
    const bool pinless = c.route == UsartRoute::none;
    if (c.mode == UsartMode::sync || c.mode == UsartMode::mspi) {
        if (c.rx_mode != UsartRxMode::normal) return false;
        if (pinless || !usart_pin(n, c.route, UsartSignal::xck).bonded) return false;
    }
    if (c.mode == UsartMode::ircom && c.rx_mode != UsartRxMode::normal) return false;
    if (c.rs485 && (pinless || !usart_pin(n, c.route, UsartSignal::xdir).bonded)) return false;
    // LBME and ODME both act on the TXD PAD - bench-measured: with
    // PORTMUX at NONE a loop-back receives nothing, because the
    // internal connection is taken at the pin, not at the shifter.
    if ((c.open_drain || c.loop_back) &&
        (pinless || !usart_pin(n, c.route, UsartSignal::txd).bonded)) return false;
    return true;
}

/// One received frame with the status bits that travelled with it.
struct UsartFrame {
    uint16_t data = 0;
    bool frame_error = false;
    bool parity_error = false;
    bool overflow = false;

    constexpr bool clean() const { return !frame_error && !parity_error && !overflow; }
};

// ---- the resource -------------------------------------------------------------

template <uint8_t n>
class Usart {
    static_assert(n < usart_count,
                  "no such USART on this package (28/32 pins: USART0..2, "
                  "48: ..USART4, 64: ..USART5)");

public:
    Usart() = delete;

    static constexpr uint8_t index = n;

    /// The event vocabulary of this instance (evsys.hpp).
    using XckEvent = EvUsartXck<n>;    ///< generator: XCK level in the host modes
    using IrdaIn = EvUsartIrda<n>;     ///< user: the IRCOM receiver's input channel

    // ---- the route table of this instance --------------------------------

    static constexpr bool has_route(UsartRoute r) { return usart_route_exists(n, r); }
    static constexpr UsartPin txd(UsartRoute r) { return usart_pin(n, r, UsartSignal::txd); }
    static constexpr UsartPin rxd(UsartRoute r) { return usart_pin(n, r, UsartSignal::rxd); }
    static constexpr UsartPin xck(UsartRoute r) { return usart_pin(n, r, UsartSignal::xck); }
    static constexpr UsartPin xdir(UsartRoute r) { return usart_pin(n, r, UsartSignal::xdir); }

    /// The route in force since the last init()/release().
    static UsartRoute route() { return route_; }

    // ---- configuration ---------------------------------------------------

    /// Compile-time form: what this package cannot bond is refused here.
    template <UsartConfig cfg>
    static void init() {
        static_assert(usart_route_exists(n, cfg.route),
                      "this package does not bond this USART's ALT1 position");
        static_assert(usart_config_valid<n>(cfg),
                      "this USART configuration is not legal on this package: a "
                      "synchronous or Host SPI mode needs an XCK pin, RS-485 needs "
                      "XDIR, and the non-normal receiver modes are asynchronous only");
        (void)init(cfg);
    }

    /// Run-time form. Stops the instance, releases the pins of the old
    /// route, routes and drives the new ones, writes the whole register
    /// set, then enables the directions asked for. False (and nothing
    /// programmed) when the config is not legal on this package.
    static bool init(const UsartConfig& cfg) {
        if (!usart_config_valid<n>(cfg)) return false;
        auto& u = regs();
        u.CTRLA = 0;                       // interrupts off before anything moves
        u.CTRLB = 0;                       // both directions off: TXD goes back to PORT
        bits_ = cfg.bits;
        mode_ = cfg.mode;
        rx_mode_ = cfg.rx_mode;
        setup_pins(cfg);
        u.BAUD = cfg.baud;
        u.CTRLC = ctrlc_byte(cfg);
        u.CTRLD = static_cast<uint8_t>(cfg.ab_window);
        u.DBGCTRL = cfg.debug_run ? USART_DBGRUN_bm : 0;
        u.EVCTRL = cfg.irda_event_input ? USART_IREI_bm : 0;
        u.TXPLCTRL = cfg.tx_pulse;         // before TXEN (27.5.14)
        u.RXPLCTRL = cfg.rx_pulse;         // before RXEN (27.5.15)
        u.STATUS = static_cast<uint8_t>(USART_TXCIF_bm | USART_RXSIF_bm |
                                        USART_ISFIF_bm | USART_BDF_bm);
        u.CTRLA = static_cast<uint8_t>(
            (cfg.loop_back ? USART_LBME_bm : 0) |
            (cfg.rs485 ? USART_RS485_bm : 0));
        u.CTRLB = static_cast<uint8_t>(
            (cfg.rx ? USART_RXEN_bm : 0) |
            (cfg.tx ? USART_TXEN_bm : 0) |
            (cfg.open_drain ? USART_ODME_bm : 0) |
            static_cast<uint8_t>(cfg.rx_mode) |
            (cfg.multiprocessor ? USART_MPCM_bm : 0));
        return true;
    }

    /// Stop the instance and hand its pins back: both directions off,
    /// interrupts off, PORTMUX to NONE, the pins this driver drove
    /// returned to inputs with their pull-up cleared. The teardown that
    /// lets another peripheral take the position (evsys.hpp's unlisten
    /// is the precedent).
    static void release() {
        auto& u = regs();
        u.CTRLA = 0;
        u.CTRLB = 0;
        release_pins();
        write_route(UsartRoute::none);
        route_ = UsartRoute::none;
    }

    // ---- directions ------------------------------------------------------

    /// RXEN. Disabling is immediate and flushes the RX buffer (27.3.2.4.2).
    static void enable_rx(bool on) { ctrlb(USART_RXEN_bm, on); }
    /// TXEN. Disabling waits for what is in flight, then TXD goes back
    /// to PORT as an INPUT (27.3.2.3.1) - a caller that wants the line
    /// held high must drive the pin itself afterwards.
    static void enable_tx(bool on) { ctrlb(USART_TXEN_bm, on); }
    static bool rx_enabled() { return (regs().CTRLB & USART_RXEN_bm) != 0; }
    static bool tx_enabled() { return (regs().CTRLB & USART_TXEN_bm) != 0; }
    /// Both directions off (the instance keeps its configuration).
    static void disable() { regs().CTRLB &= static_cast<uint8_t>(~(USART_RXEN_bm | USART_TXEN_bm)); }

    /// Repeated reads of the data registers until RXCIF clears
    /// (27.3.2.4.3) - the documented flush.
    static void flush_rx() {
        for (uint8_t i = 0; i < 8 && (regs().STATUS & USART_RXCIF_bm) != 0; ++i) {
            (void)receive();
        }
    }

    // ---- frame format and baud -------------------------------------------

    /// Rewrite CTRLC's frame half, keeping the communication mode.
    /// Nothing may be in flight (27.3.1).
    static void frame(const UsartFormat& f) {
        bits_ = f.bits;
        regs().CTRLC = static_cast<uint8_t>(
            static_cast<uint8_t>(mode_) | static_cast<uint8_t>(f.parity) |
            (f.two_stop ? USART_SBMODE_bm : 0) | static_cast<uint8_t>(f.bits));
    }
    static UsartBits bits() { return bits_; }
    static UsartMode mode() { return mode_; }

    static uint16_t baud_reg() { return regs().BAUD; }
    /// Writing BAUD updates the prescaler at once and corrupts anything
    /// in flight (27.5.10).
    static void baud_reg(uint16_t v) { regs().BAUD = v; }

    /// Set the rate from a peripheral clock; false (and BAUD untouched)
    /// when the register cannot express it.
    static bool set_baud(uint32_t clk_per_hz, uint32_t baud) {
        const uint16_t r = usart_baud_reg(clk_per_hz, baud, samples());
        if (r == 0) return false;
        regs().BAUD = r;
        return true;
    }
    /// What the generator really produces at this clock (the fractional
    /// divisor read back from the register).
    static uint32_t actual_baud(uint32_t clk_per_hz) {
        return usart_actual_baud(clk_per_hz, regs().BAUD, samples());
    }
    /// Samples per bit for the mode and receiver mode in force.
    static uint8_t samples() { return usart_samples(mode_, rx_mode_); }

    /// The XCK ceilings of the synchronous modes: a host divides
    /// CLK_PER by at least two (S = 2, divisor >= 1); a client must
    /// sample the incoming clock twice per edge, so its XCK must stay
    /// strictly below CLK_PER/4 (27.3.3.1.2).
    static constexpr uint32_t max_host_xck_hz(uint32_t clk_per_hz) { return clk_per_hz / 2; }
    static constexpr uint32_t max_client_xck_hz(uint32_t clk_per_hz) { return clk_per_hz / 4; }

    // ---- CTRLB knobs -----------------------------------------------------

    static void rx_mode(UsartRxMode m) {
        rx_mode_ = m;
        regs().CTRLB = static_cast<uint8_t>((regs().CTRLB & ~USART_RXMODE_gm) |
                                            static_cast<uint8_t>(m));
    }
    static UsartRxMode rx_mode() {
        return static_cast<UsartRxMode>(regs().CTRLB & USART_RXMODE_gm);
    }
    /// MPCM: with 9 data bits the ninth bit, with 5..8 the first stop
    /// bit, marks an address frame; data frames are dropped until one
    /// arrives (27.3.4.3).
    static void multiprocessor(bool on) { ctrlb(USART_MPCM_bm, on); }
    static bool multiprocessor() { return (regs().CTRLB & USART_MPCM_bm) != 0; }
    /// ODME - always with the errata work around on the pin: see
    /// setup_pins().
    static void open_drain(bool on) { ctrlb(USART_ODME_bm, on); }
    /// LBME: TXD feeds the receiver, RXD is released to other peripherals.
    static void loop_back(bool on) { ctrla(USART_LBME_bm, on); }
    static void rs485(bool on) { ctrla(USART_RS485_bm, on); }

    /// Errata 2.16.2 / 2.15.2: SFDEN must NOT stay set while the device
    /// is in Active mode - a read of RXDATA during a reception re-arms
    /// the detector and the frame restarts corrupted. So: arm it right
    /// before going to standby, disarm it on the way back, and only
    /// where the protocol guarantees no frame is already arriving.
    static void arm_start_of_frame() { ctrlb(USART_SFDEN_bm, true); }
    static void disarm_start_of_frame() { ctrlb(USART_SFDEN_bm, false); }
    static bool start_of_frame_armed() { return (regs().CTRLB & USART_SFDEN_bm) != 0; }

    // ---- the remaining registers -----------------------------------------

    static void auto_baud_window(UsartAbWindow w) { regs().CTRLD = static_cast<uint8_t>(w); }
    static UsartAbWindow auto_baud_window() { return static_cast<UsartAbWindow>(regs().CTRLD); }
    /// EVCTRL.IREI: the IRCOM receiver takes its input from an event
    /// channel instead of RXD (27.3.3.2.7).
    static void irda_event_input(bool on) { regs().EVCTRL = on ? USART_IREI_bm : 0; }
    template <uint8_t ch>
    static void irda_on(EventChannel<ch> c) {
        IrdaIn::listen(c);
        regs().EVCTRL = USART_IREI_bm;
    }
    /// TXPLCTRL: 0 = 3/16 of a bit, 1..0xFE = that many CLK_PER, 0xFF =
    /// no pulse coding (the pass-through that makes IRCOM a plain
    /// half-duplex/loop-back path). Write it before TXEN.
    static void tx_pulse(uint8_t v) { regs().TXPLCTRL = v; }
    static uint8_t tx_pulse() { return regs().TXPLCTRL; }
    /// RXPLCTRL: 0 = no filter, else RXPL + 1 samples of pulse are
    /// needed for a logical zero. Write it before RXEN.
    static void rx_pulse(uint8_t v) { regs().RXPLCTRL = static_cast<uint8_t>(v & USART_RXPL_gm); }
    static uint8_t rx_pulse() { return regs().RXPLCTRL; }
    static void debug_run(bool on) { regs().DBGCTRL = on ? USART_DBGRUN_bm : 0; }

    // ---- status (27.5.5) --------------------------------------------------

    static uint8_t status() { return regs().STATUS; }
    static bool rxc_flag() { return (regs().STATUS & USART_RXCIF_bm) != 0; }
    static bool dre_flag() { return (regs().STATUS & USART_DREIF_bm) != 0; }
    static bool txc_flag() { return (regs().STATUS & USART_TXCIF_bm) != 0; }
    static bool rxs_flag() { return (regs().STATUS & USART_RXSIF_bm) != 0; }
    static bool isf_flag() { return (regs().STATUS & USART_ISFIF_bm) != 0; }
    static bool break_flag() { return (regs().STATUS & USART_BDF_bm) != 0; }

    // The write-one-to-clear half of STATUS: PLAIN stores of the single
    // bit (an RMW would write back every flag it read and clear them
    // all) - the pin.hpp INTFLAGS discipline.
    /// ISR body for USARTn_TXC_vect.
    [[gnu::always_inline]] static void clear_txc() { regs().STATUS = USART_TXCIF_bm; }
    static void clear_rxs() { regs().STATUS = USART_RXSIF_bm; }
    static void clear_isf() { regs().STATUS = USART_ISFIF_bm; }
    static void clear_break() { regs().STATUS = USART_BDF_bm; }
    /// STATUS.WFB: treat the next incoming frame as a break of any
    /// length (GENAUTO only; it disarms itself after that frame). The
    /// bit is WRITE-ONLY (27.5.5) - there is no reading back whether an
    /// arming is still pending, so no verb offers to.
    static void wait_for_break() { regs().STATUS = USART_WFB_bm; }

    /// Errata 2.16.3: once ISFIF is set the receiver is dead, and
    /// clearing the flag does not revive it - RXEN must go 0 then 1.
    /// Every auto-baud path calls this after seeing the flag.
    static void recover_from_isf() {
        clear_isf();
        auto& u = regs();
        const uint8_t b = u.CTRLB;
        u.CTRLB = static_cast<uint8_t>(b & ~USART_RXEN_bm);
        u.CTRLB = static_cast<uint8_t>(b | USART_RXEN_bm);
    }

    // ---- interrupts (27.5.6) ---------------------------------------------

    static void enable_rxc_interrupt(bool on) { ctrla(USART_RXCIE_bm, on); }
    static void enable_txc_interrupt(bool on) { ctrla(USART_TXCIE_bm, on); }
    static void enable_dre_interrupt(bool on) { ctrla(USART_DREIE_bm, on); }
    /// RXSIE shares the RXC vector: it fires on a start bit detected in
    /// standby (with SFDEN armed).
    static void enable_rxs_interrupt(bool on) { ctrla(USART_RXSIE_bm, on); }
    /// ABEIE shares the RXC vector too: it fires on ISFIF.
    static void enable_autobaud_error_interrupt(bool on) { ctrla(USART_ABEIE_bm, on); }

    // ---- data -------------------------------------------------------------

    /// Read the frame at the head of the RX FIFO with its status, in
    /// the order the character size demands. Compile-time character
    /// size: no branch in the ISR. ISR body for USARTn_RXC_vect.
    template <UsartBits b>
    [[gnu::always_inline]] static UsartFrame receive_as() {
        if constexpr (b == UsartBits::nine_low_first) {
            // RXDATAH is the shifting read here: the low byte first.
            const uint8_t lo = regs().RXDATAL;
            const uint8_t hi = regs().RXDATAH;
            return frame_of(hi, lo);
        } else {
            const uint8_t hi = regs().RXDATAH;   // status of THIS frame
            const uint8_t lo = regs().RXDATAL;   // shifts the FIFO
            return frame_of(hi, lo);
        }
    }
    /// Same, honouring the character size configured by init().
    static UsartFrame receive() {
        return bits_ == UsartBits::nine_low_first ? receive_as<UsartBits::nine_low_first>()
                                                  : receive_as<UsartBits::eight>();
    }

    /// Load one frame into the transmit buffer, in the order the
    /// character size demands. DREIF must be set (27.5.3).
    template <UsartBits b>
    [[gnu::always_inline]] static void transmit_as(uint16_t v) {
        if constexpr (b == UsartBits::nine_low_first) {
            regs().TXDATAL = static_cast<uint8_t>(v);
            regs().TXDATAH = static_cast<uint8_t>((v >> 8) & USART_DATA8_bm);   // shifts
        } else if constexpr (b == UsartBits::nine_high_first) {
            regs().TXDATAH = static_cast<uint8_t>((v >> 8) & USART_DATA8_bm);
            regs().TXDATAL = static_cast<uint8_t>(v);                            // shifts
        } else {
            regs().TXDATAL = static_cast<uint8_t>(v);
        }
    }
    /// Same, honouring the character size configured by init().
    static void transmit(uint16_t v) {
        switch (bits_) {
            case UsartBits::nine_low_first: transmit_as<UsartBits::nine_low_first>(v); break;
            case UsartBits::nine_high_first: transmit_as<UsartBits::nine_high_first>(v); break;
            default: transmit_as<UsartBits::eight>(v); break;
        }
    }

    // ---- polled convenience (the non-interrupt tasks) ---------------------

    /// Wait (bounded) for room in the transmit buffer, then load a
    /// frame. False when the spin budget ran out - a caller that wants
    /// to block forever polls dre_flag() itself.
    static bool send(uint16_t v, uint32_t spins = 500'000u) {
        while (!dre_flag()) {
            if (spins-- == 0) return false;
        }
        transmit(v);
        return true;
    }
    /// One frame if the receiver has one.
    static std::optional<UsartFrame> poll() {
        if (!rxc_flag()) return {};
        return receive();
    }
    /// Wait (bounded) for a frame.
    static std::optional<UsartFrame> wait(uint32_t spins = 500'000u) {
        while (!rxc_flag()) {
            if (spins-- == 0) return {};
        }
        return receive();
    }
    /// Wait (bounded) for the line to go idle, then CLEAR TXCIF - so
    /// what a later call reports is the NEXT transmission finishing, not
    /// this one. Two calls in a row therefore do not both return at
    /// once: the second spins out its whole budget (bench: a half-duplex
    /// turnaround that waited here twice stayed deaf for 60 ms and lost
    /// the answer). One call per burst.
    static bool wait_line_idle(uint32_t spins = 500'000u) {
        while (!txc_flag()) {
            if (spins-- == 0) return false;
        }
        clear_txc();
        return true;
    }

    // ---- registers and pins ----------------------------------------------

    static constexpr USART_t& regs() {
        if constexpr (n == 0) return USART0;
        else if constexpr (n == 1) return USART1;
        else if constexpr (n == 2) return USART2;
#ifdef USART3
        else if constexpr (n == 3) return USART3;
#endif
#ifdef USART4
        else if constexpr (n == 4) return USART4;
#endif
#ifdef USART5
        else if constexpr (n == 5) return USART5;
#endif
        else return USART0;
    }

    /// PORTMUX position of this instance's two route bits (the device
    /// header's own group positions).
    static constexpr uint8_t route_gp() {
        if constexpr (n == 0) return PORTMUX_USART0_gp;
        else if constexpr (n == 1) return PORTMUX_USART1_gp;
        else if constexpr (n == 2) return PORTMUX_USART2_gp;
#ifdef USART3
        else if constexpr (n == 3) return PORTMUX_USART3_gp;
#endif
#ifdef USART4
        else if constexpr (n == 4) return PORTMUX_USART4_gp;
#endif
#ifdef USART5
        else if constexpr (n == 5) return PORTMUX_USART5_gp;
#endif
        else return 0;
    }

    /// USARTROUTEA holds USART0..3, USARTROUTEB USART4..5 (the second
    /// register only exists on the packages that have those instances).
    static volatile uint8_t& route_reg() {
#ifdef USART4
        if constexpr (n > 3) return PORTMUX.USARTROUTEB;
        else return PORTMUX.USARTROUTEA;
#else
        return PORTMUX.USARTROUTEA;
#endif
    }

    static void write_route(UsartRoute r) {
        constexpr uint8_t gp = route_gp();
        constexpr uint8_t gm = static_cast<uint8_t>(0x03u << gp);
        volatile uint8_t& reg = route_reg();
        reg = static_cast<uint8_t>((reg & ~gm) |
                                   static_cast<uint8_t>(static_cast<uint8_t>(r) << gp));
    }

    /// The route bits as they read back.
    static UsartRoute routed() {
        constexpr uint8_t gp = route_gp();
        constexpr uint8_t gm = static_cast<uint8_t>(0x03u << gp);
        return static_cast<UsartRoute>((route_reg() & gm) >> gp);
    }

private:
    static constexpr uint8_t ctrlc_byte(const UsartConfig& c) {
        if (c.mode == UsartMode::mspi) {
            return static_cast<uint8_t>(USART_CMODE_MSPI_gc |
                                        (c.lsb_first ? USART_UDORD_bm : 0) |
                                        (c.sample_trailing ? USART_UCPHA_bm : 0));
        }
        return static_cast<uint8_t>(static_cast<uint8_t>(c.mode) |
                                    static_cast<uint8_t>(c.parity) |
                                    (c.two_stop ? USART_SBMODE_bm : 0) |
                                    static_cast<uint8_t>(c.bits));
    }

    static constexpr UsartFrame frame_of(uint8_t hi, uint8_t lo) {
        return {static_cast<uint16_t>(((hi & USART_DATA8_bm) ? 0x100u : 0u) | lo),
                (hi & USART_FERR_bm) != 0,
                (hi & USART_PERR_bm) != 0,
                (hi & USART_BUFOVF_bm) != 0};
    }

    static void ctrla(uint8_t bit, bool on) {
        if (on) regs().CTRLA |= bit; else regs().CTRLA &= static_cast<uint8_t>(~bit);
    }
    static void ctrlb(uint8_t bit, bool on) {
        if (on) regs().CTRLB |= bit; else regs().CTRLB &= static_cast<uint8_t>(~bit);
    }

    /// Route, then give PORT the directions the mode needs. Runs off
    /// the route table (a run-time port/pin lookup, pin.hpp's
    /// port_by_letter / pinctrl_of), so a position this package lacks
    /// never instantiates a Pin that does not exist.
    static void setup_pins(const UsartConfig& c) {
        release_pins();
        write_route(c.route);
        route_ = c.route;
        if (c.route == UsartRoute::none) return;
        const char p = usart_port_letter(n);
        volatile PORT_t& port = port_by_letter(p);
        const UsartPin t = usart_pin(n, c.route, UsartSignal::txd);
        const UsartPin r = usart_pin(n, c.route, UsartSignal::rxd);
        const UsartPin x = usart_pin(n, c.route, UsartSignal::xck);
        const UsartPin d = usart_pin(n, c.route, UsartSignal::xdir);
        if (c.tx && t.bonded) {
            if (c.open_drain) {
                // Errata 2.16.1 / 2.15.1: an OUTPUT TXD drives high
                // whatever ODME says. The work around is to leave the
                // direction at input and let the pull-up hold the line;
                // the transmitter still pulls it low through the pad.
                port.DIRCLR = static_cast<uint8_t>(1u << t.pin);
                pinctrl_of(p, t.pin) |= PORT_PULLUPEN_bm;
                driven_ = static_cast<uint8_t>(driven_ | (1u << t.pin));
            } else {
                port.OUTSET = static_cast<uint8_t>(1u << t.pin);   // idle high first
                port.DIRSET = static_cast<uint8_t>(1u << t.pin);
                driven_ = static_cast<uint8_t>(driven_ | (1u << t.pin));
            }
        }
        // In loop-back the receiver listens to TXD and RXD is free for
        // another peripheral: do not claim it.
        if (c.rx && r.bonded && !c.loop_back) {
            port.DIRCLR = static_cast<uint8_t>(1u << r.pin);
        }
        if (x.bonded && (c.mode == UsartMode::sync || c.mode == UsartMode::mspi)) {
            if (c.sync_client) {
                port.DIRCLR = static_cast<uint8_t>(1u << x.pin);
            } else {
                port.DIRSET = static_cast<uint8_t>(1u << x.pin);
                driven_ = static_cast<uint8_t>(driven_ | (1u << x.pin));
            }
        }
        if (c.rs485 && d.bonded) {
            port.OUTCLR = static_cast<uint8_t>(1u << d.pin);       // idle low
            port.DIRSET = static_cast<uint8_t>(1u << d.pin);
            driven_ = static_cast<uint8_t>(driven_ | (1u << d.pin));
        }
    }

    /// Everything this driver drove goes back to being an input with
    /// its pull-up cleared.
    static void release_pins() {
        if (driven_ == 0) return;
        const char p = usart_port_letter(n);
        volatile PORT_t& port = port_by_letter(p);
        port.DIRCLR = driven_;
        for (uint8_t i = 0; i < 8; ++i) {
            if (driven_ & (1u << i)) {
                pinctrl_of(p, i) &= static_cast<uint8_t>(~PORT_PULLUPEN_bm);
            }
        }
        driven_ = 0;
    }

    static inline UsartRoute route_ = UsartRoute::none;
    static inline UsartBits bits_ = UsartBits::eight;
    static inline UsartMode mode_ = UsartMode::async;
    static inline UsartRxMode rx_mode_ = UsartRxMode::normal;
    static inline uint8_t driven_ = 0;    ///< pins this driver set as outputs
};

// ---- tasks --------------------------------------------------------------------

/*
 * Uart<n, route, rx_size, tx_size>
 *
 * The interrupt-driven full-duplex byte transport every console sits
 * on: 8N1 by default, SPSC rings on both sides, ByteSink + ByteSource.
 *
 *  - all state is `static inline` (one per instantiation, .bss, no ctor
 *    running before main): hardware is touched ONLY by the explicit
 *    init(), called after the clock is up;
 *  - byte transport only - text formatting lives in print.hpp;
 *  - RX hardware error flags (frame / parity / buffer overflow) are
 *    counted; corrupted bytes (FERR/PERR) are dropped.
 *
 * The empty default constructor is intentionally AVAILABLE: an instance
 * carries no state and acts as a zero-cost tag so call sites can read
 * naturally, e.g.
 *
 *   using Serial = brio::Uart<2, brio::Route::alt1>;
 *   constexpr Serial serial;                 // tag object (no state)
 *
 *   ISR(USART2_RXC_vect) { Serial::rxc(); }
 *   ISR(USART2_DRE_vect) { Serial::dre(); }
 *
 *   using SysClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
 *   constexpr SysClock clock;
 *   int main() {
 *       SysClock::init();
 *       Serial::init(clock, 460800);
 *       sei();
 *       brio::print(serial, "hello", brio::crlf);
 *   }
 *
 * TX policy: write_byte() has TRY semantics (false when the TX ring is
 * full, nothing counted - the caller decides whether to retry, drop or
 * block; print.hpp blocks). RX overflow (ring full, byte lost) IS
 * counted, as are the hardware error flags.
 */
// Ring defaults sized for console-class traffic AND for lock-free rings:
// both <= 256 keeps Ring's index_t at 8 bits, which on AVR (atomic_width
// 1) means no interrupt masking at all on either side; a 512-byte ring
// would silently switch to the critical-section path (~10 cycles and
// added interrupt latency per byte). RX 64 absorbs ~1.4 ms of full-rate
// 460800 traffic - plenty against dispatches that last microseconds.
// Streaming apps may still ask for more.
template <int usart_num, Route route = Route::def,
          uint32_t rx_size = 64, uint32_t tx_size = 256>
class Uart {
    static_assert(usart_num >= 0 && usart_num <= 5, "usart_num must be 0..5");
    using U = Usart<static_cast<uint8_t>(usart_num)>;
    static_assert(usart_route_exists(static_cast<uint8_t>(usart_num), route),
                  "this package does not bond this USART's ALT1 position");

    // One ring pair per instantiation (static inline -> .bss, no ctor).
    static inline Ring<uint8_t, rx_size, AvrPlatform> m_rx{};
    static inline Ring<uint8_t, tx_size, AvrPlatform> m_tx{};

    // Error counters, written in ISRs, read from the main loop. uint8_t
    // reads/writes are atomic on AVR; they wrap at 255. Written as
    // `x = x + 1` because compound ops on volatile are deprecated in C++20.
    static inline volatile uint8_t m_rx_overruns = 0;   // RX ring full, byte lost
    static inline volatile uint8_t m_frame_errors = 0;  // FERR: byte dropped
    static inline volatile uint8_t m_parity_errors = 0; // PERR: byte dropped
    static inline volatile uint8_t m_hw_overruns = 0;   // BUFOVF: bytes lost in HW
    static inline uint32_t m_baud = 0;                   // for rebase()

public:
    /// Instances are empty tags for concept-based call sites (print(serial,...)).
    constexpr Uart() = default;

    /// The resource underneath, for the register-level verbs a console
    /// occasionally wants (route teardown, DBGRUN, the status flags).
    using Resource = U;

    // ---- lifecycle -------------------------------------------------------

    /**
     * @brief Configure pins, PORTMUX, baud rate and enable the USART.
     *
     * Call AFTER the main clock is set up and before sei(); `clock` is
     * the app's brio::Clock tag (avrdx/clock.hpp): the baud divisor is
     * derived from Clock::hz, never from F_CPU. With a constant baud the
     * divisor folds to a constant. The RXC interrupt is enabled here; the
     * DRE interrupt is enabled on demand by write_byte().
     */
    template <typename Clock>
    static void init(Clock clock, uint32_t baud) {
        static_assert(clock_follows<Clock, Uart>(),
                      "this Uart is initialized with a DynamicClock that does not "
                      "list it among its Users: it would keep the old baud after "
                      "a clock change");
        m_baud = baud;
        // init() STARTS the transport: whatever a previous life of this
        // instance left in the rings and the counters is not this one's
        // traffic. (Bench: a suite that re-init'ed the same Uart after
        // driving the resource directly read a stale byte as the first
        // one of the new session.)
        while (m_rx.pop()) {
        }
        while (m_tx.pop()) {
        }
        clear_errors();
        U::template init<UsartConfig{.route = route}>();
        U::baud_reg(usart_baud_reg(clock_hz(clock), baud));
        U::enable_rxc_interrupt(true);

        // Hold TX idle-high so the first start bit is clean. Pre-kernel
        // init: the one place a millisecond busy-wait is legitimate.
        delay_us(clock, 10'000);
    }

    /**
     * @brief The peripheral clock changed (DynamicClock fan-out): keep
     * the same baud rate at the new rate.
     *
     * Waits for the TX side to go idle first (ring drained by the DRE
     * ISR, shifter empty: TXCIF), so nothing already queued is sent at
     * a half-changed rate; a byte being RECEIVED during the switch may
     * be garbled - the caller picks a quiet moment. Main context only.
     */
    static void rebase(uint32_t hz) {
        // Called BEFORE the clock actually changes (DynamicClock::set
        // fans out first, then switches), so the drain below runs at the
        // rate the bytes were queued for.
        while (!m_tx.empty()) {          // the DRE ISR keeps draining
        }
        while (!U::dre_flag()) {         // last byte in the shifter
        }
        // TWO frame times (10 bits each) at the CURRENT rate, recovered
        // from the BAUD register, let the shifter finish: DREIF says the
        // buffer is empty, the byte in the shifter may have just started
        // (one frame) and the measure of "now" is loose (the second).
        // Bench: one frame plus 1 us corrupted the last byte at several
        // rates. TXCIF is not used: it is stale-set by any earlier idle
        // period and cannot tell "done" from "still shifting".
        const uint32_t old_hz = static_cast<uint32_t>(U::baud_reg()) * m_baud / 4u;
        delay_us_runtime(cycles_per_us(old_hz), 2u * (10'000'000u / m_baud) + 2u);
        U::baud_reg(usart_baud_reg(hz, m_baud));
    }

    /// The minimum usable rate for a baud: BAUD must be >= 64 (16 * baud
    /// per the normal-speed formula). rebase() to a slower clock than
    /// this leaves the USART unable to hit the baud - check before.
    static constexpr uint32_t min_hz_for(uint32_t baud) { return usart_min_clock_hz(baud); }

    /// True if the USART can produce `baud` at `hz` (BAUD >= 64).
    static constexpr bool can_baud(uint32_t hz, uint32_t baud) {
        return usart_baud_reg(hz, baud) != 0;
    }

    /// What the fractional generator really produces at `hz`.
    static uint32_t actual_baud(uint32_t hz) {
        return usart_actual_baud(hz, U::baud_reg());
    }

    // ---- ISR bodies ------------------------------------------------------

    /**
     * @brief RX Complete interrupt body - call from ISR(USARTn_RXC_vect).
     *
     * RXDATAH (status for the byte at the FIFO head) is read BEFORE
     * RXDATAL (which advances the FIFO). Corrupted bytes (frame/parity) are
     * counted and dropped; BUFOVF means the hardware already lost bytes.
     *
     * @return true when the RX ring transitioned empty -> non-empty: the
     * edge signal for kernel glue ("post RxActivity to the serial AO on
     * true"). Every empty->non-empty transition reports true and the
     * consumer only empties the ring by draining it, so no wakeup is ever
     * lost. Plain (non-kernel) apps may ignore the return value.
     */
    // always_inline: single call site (the ISR binding) - see ticker.hpp
    // pit() for the register-set rationale.
    [[gnu::always_inline]] static bool rxc() {
        const UsartFrame f = U::template receive_as<UsartBits::eight>();
        if (f.overflow) {
            m_hw_overruns = m_hw_overruns + 1;
        }
        if (f.frame_error || f.parity_error) {
            if (f.frame_error) m_frame_errors = m_frame_errors + 1;
            if (f.parity_error) m_parity_errors = m_parity_errors + 1;
            return false;  // drop the corrupted byte
        }
        const bool was_empty = m_rx.empty();
        if (!m_rx.push(static_cast<uint8_t>(f.data))) {
            m_rx_overruns = m_rx_overruns + 1;
            return false;  // full ring cannot be empty: no edge
        }
        return was_empty;
    }

    /**
     * @brief Data Register Empty interrupt body - call from ISR(USARTn_DRE_vect).
     *
     * Feeds the next byte from the TX ring; disables itself when the ring
     * drains (write_byte() re-enables it).
     */
    // always_inline: single call site (the ISR binding) - see ticker.hpp
    // pit() for the register-set rationale.
    [[gnu::always_inline]] static void dre() {
        if (const auto c = m_tx.pop()) {
            U::template transmit_as<UsartBits::eight>(*c);
            if (m_tx.empty()) {
                U::regs().CTRLA &= static_cast<uint8_t>(~USART_DREIE_bm);
            }
        } else {
            U::regs().CTRLA &= static_cast<uint8_t>(~USART_DREIE_bm);
        }
    }

    // ---- byte transport (satisfies ByteSink / ByteSource) ----------------

    /// Try to queue one byte for transmission; false when the TX ring is full.
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            return false;
        }
        U::regs().CTRLA |= USART_DREIE_bm;
        return true;
    }

    /// Fetch one received byte; false when nothing is pending.
    static bool read_byte(uint8_t &b) {
        const auto v = m_rx.pop();
        if (!v) {
            return false;
        }
        b = *v;
        return true;
    }

    /// Queue as much of the buffer as fits; returns the number queued.
    static uint8_t write(const uint8_t *buffer, uint8_t len) {
        uint8_t written = 0;
        while (written < len && write_byte(buffer[written])) {
            ++written;
        }
        return written;
    }

    // ---- introspection ---------------------------------------------------

    static auto rx_pending() { return m_rx.count(); }
    static bool tx_idle() { return m_tx.empty(); }

    static uint8_t rx_overruns() { return m_rx_overruns; }
    static uint8_t frame_errors() { return m_frame_errors; }
    static uint8_t parity_errors() { return m_parity_errors; }
    static uint8_t hw_overruns() { return m_hw_overruns; }

    static void clear_errors() {
        m_rx_overruns = 0;
        m_frame_errors = 0;
        m_parity_errors = 0;
        m_hw_overruns = 0;
    }
};

static_assert(ByteTransport<Uart<0>>, "Uart must satisfy the transport concepts");

/// The polled half-duplex and synchronous tasks share this much: a
/// configured resource, a bounded send/receive pair, and a rebase that
/// recomputes BAUD from the rate the task was born with.
template <uint8_t n, UsartRoute route, UsartMode task_mode>
struct UsartTaskBase {
    UsartTaskBase() = delete;
    using Resource = Usart<n>;

    /// Does this package bond everything this task's route needs?
    static constexpr bool available =
        usart_route_exists(n, route) &&
        (task_mode == UsartMode::async || task_mode == UsartMode::ircom ||
         (route != UsartRoute::none && usart_pin(n, route, UsartSignal::xck).bonded));

    static bool send(uint16_t v, uint32_t spins = 500'000u) { return Resource::send(v, spins); }
    static std::optional<UsartFrame> poll() { return Resource::poll(); }
    static std::optional<UsartFrame> wait(uint32_t spins = 500'000u) { return Resource::wait(spins); }
    static bool wait_line_idle(uint32_t spins = 500'000u) { return Resource::wait_line_idle(spins); }
    static void release() { Resource::release(); }

    /// ClockUser: keep the rate the task was asked for. The transmitter
    /// must be idle - writing BAUD corrupts what is in flight (27.5.10).
    static void rebase(uint32_t hz) {
        if (baud_ == 0) return;
        (void)Resource::set_baud(hz, baud_);
    }
    static uint32_t requested_baud() { return baud_; }
    static uint32_t actual_baud(uint32_t hz) { return Resource::actual_baud(hz); }

protected:
    static inline uint32_t baud_ = 0;
};

/**
 * OneWire<n, route>: half duplex on a single TXD/RXD line (27.3.3.2.6).
 * LBME connects the transmitter to the receiver internally and frees
 * the RXD pin; ODME keeps the transmitter from ever driving the line
 * high, and the pin's pull-up holds it idle. Errata 2.16.1 / 2.15.1 is
 * why init() leaves TXD's DIRECTION at input.
 *
 * Because the transmitter's own frames come back through the receiver,
 * every send is echoed: echo() reads that copy back, and a mismatch is
 * a collision with another device on the line (27.3.3.2.6). The
 * turnaround is explicit: talk() before transmitting, listen() after
 * the line goes idle.
 *
 * TURNAROUND GUARD. RXCIF is raised at the MIDDLE of the stop bit, half
 * a bit BEFORE the sender's own TXCIF. A device that answers the instant
 * it has the frame therefore starts its start bit while the other end is
 * still transmitting - and any half-duplex scheme that stops listening
 * while it talks loses the beginning of that answer (bench: two bits
 * swallowed at 2400 baud, the receiver then locking onto a later low).
 * A responder must wait a few bit times before taking the line.
 */
template <uint8_t n, UsartRoute route = UsartRoute::def>
struct OneWire : UsartTaskBase<n, route, UsartMode::async> {
    using Base = UsartTaskBase<n, route, UsartMode::async>;
    using Resource = Usart<n>;

    /// A one-wire line needs a real TXD pin.
    static constexpr bool available =
        usart_route_exists(n, route) && route != UsartRoute::none &&
        usart_pin(n, route, UsartSignal::txd).bonded;

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, UsartFormat fmt = {}) {
        static_assert(clock_follows<Clock, OneWire>(),
                      "OneWire on a DynamicClock that does not list it: its baud would go stale");
        if constexpr (!available) {
            (void)clock; (void)baud; (void)fmt;
            return false;
        } else {
            const uint16_t r = usart_baud_reg(clock_hz(clock), baud);
            if (r == 0) return false;
            Base::baud_ = baud;
            return Resource::init({.route = route, .bits = fmt.bits, .parity = fmt.parity,
                                   .two_stop = fmt.two_stop, .baud = r,
                                   .loop_back = true, .open_drain = true});
        }
    }

    /// The line's own pin, for an app that wants to look at it.
    static constexpr UsartPin line() { return usart_pin(n, route, UsartSignal::txd); }

    /// Turnaround: claim the line (the transmitter can pull it low).
    static void talk() { Resource::enable_tx(true); }
    /// Turnaround: let go (the pull-up holds the line, other devices
    /// may drive it). Waits for the last frame to leave first.
    static bool listen(uint32_t spins = 500'000u) {
        const bool idle = Resource::wait_line_idle(spins);
        Resource::enable_tx(false);
        return idle;
    }

    /// The copy of a transmitted frame the receiver saw: equal means
    /// nobody else was pulling the line.
    static bool echo_matches(uint8_t sent, uint32_t spins = 500'000u) {
        const auto f = Resource::wait(spins);
        return f && f->clean() && static_cast<uint8_t>(f->data) == sent;
    }
};

/**
 * Rs485<n, route>: the transport plus the automatic drive enable
 * (27.3.3.2.6). XDIR goes high one baud clock BEFORE the start bit -
 * the guard time an external line driver needs - and stays high through
 * the stop bit(s); the hardware does all of it, so the task's only job
 * is to claim the XDIR pin and refuse the routes that have none.
 * `one_wire` adds LBME, the "RS-485 with loop-back" wiring of figure
 * 27-12 (TXD to the driver, receiver fed from TXD, RXD free).
 */
template <uint8_t n, UsartRoute route = UsartRoute::def>
struct Rs485 : UsartTaskBase<n, route, UsartMode::async> {
    using Base = UsartTaskBase<n, route, UsartMode::async>;
    using Resource = Usart<n>;

    static constexpr bool available =
        usart_route_exists(n, route) && route != UsartRoute::none &&
        usart_pin(n, route, UsartSignal::xdir).bonded;

    /// XDIR leads the frame by one baud clock (27.3.3.2.6, figure 27-11).
    static constexpr uint8_t guard_bits = 1;

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, UsartFormat fmt = {}, bool one_wire = false) {
        static_assert(clock_follows<Clock, Rs485>(),
                      "Rs485 on a DynamicClock that does not list it: its baud would go stale");
        if constexpr (!available) {
            (void)clock; (void)baud; (void)fmt; (void)one_wire;
            return false;
        } else {
            const uint16_t r = usart_baud_reg(clock_hz(clock), baud);
            if (r == 0) return false;
            Base::baud_ = baud;
            return Resource::init({.route = route, .bits = fmt.bits, .parity = fmt.parity,
                                   .two_stop = fmt.two_stop, .baud = r,
                                   .loop_back = one_wire, .open_drain = one_wire,
                                   .rs485 = true});
        }
    }

    static constexpr UsartPin drive_enable() { return usart_pin(n, route, UsartSignal::xdir); }
};

/**
 * SyncHost<n, route>: synchronous USART with XCK driven out (27.3.3.1).
 * The rate is XCK itself (S = 2, so at most CLK_PER/2) and only
 * BAUD[15:6] counts. The sampling edge is the pin's business: with
 * INVEN clear the rising edge starts a bit and RXD is sampled on the
 * falling one, with INVEN set the two swap (figure 27-4) - invert_xck()
 * is that switch.
 */
template <uint8_t n, UsartRoute route = UsartRoute::def>
struct SyncHost : UsartTaskBase<n, route, UsartMode::sync> {
    using Base = UsartTaskBase<n, route, UsartMode::sync>;
    using Resource = Usart<n>;
    static constexpr bool available = Base::available;

    template <typename Clock>
    static bool init(Clock clock, uint32_t xck_hz, UsartFormat fmt = {}) {
        static_assert(clock_follows<Clock, SyncHost>(),
                      "SyncHost on a DynamicClock that does not list it: its XCK would go stale");
        if constexpr (!available) {
            (void)clock; (void)xck_hz; (void)fmt;
            return false;
        } else {
            const uint32_t hz = clock_hz(clock);
            if (xck_hz > Resource::max_host_xck_hz(hz)) return false;
            const uint16_t r = usart_baud_reg(hz, xck_hz, 2);
            if (r == 0) return false;
            Base::baud_ = xck_hz;
            return Resource::init({.route = route, .mode = UsartMode::sync,
                                   .bits = fmt.bits, .parity = fmt.parity,
                                   .two_stop = fmt.two_stop, .baud = r});
        }
    }
    static void rebase(uint32_t hz) {
        if (Base::baud_ == 0) return;
        const uint16_t r = usart_baud_reg(hz, Base::baud_, 2);
        if (r != 0) Resource::baud_reg(r);
    }

    static constexpr UsartPin clock_pin() { return usart_pin(n, route, UsartSignal::xck); }
    /// INVEN on XCK: which edge transmits and which samples.
    static void invert_xck(bool on) {
        if constexpr (available) {
            constexpr UsartPin x = usart_pin(n, route, UsartSignal::xck);
            volatile uint8_t& c = pinctrl_of(x.port, x.pin);
            if (on) c |= PORT_INVEN_bm; else c &= static_cast<uint8_t>(~PORT_INVEN_bm);
        } else {
            (void)on;
        }
    }
};

/**
 * SyncClient<n, route>: synchronous USART taking XCK in (27.3.3.1.2).
 * BAUD has no effect - the host sets the pace - but the incoming clock
 * must be sampled twice per edge, so it has to stay strictly below
 * CLK_PER/4. Not a ClockUser: nothing here follows CLK_PER.
 */
template <uint8_t n, UsartRoute route = UsartRoute::def>
struct SyncClient : UsartTaskBase<n, route, UsartMode::sync> {
    using Base = UsartTaskBase<n, route, UsartMode::sync>;
    using Resource = Usart<n>;
    static constexpr bool available = Base::available;

    static bool init(UsartFormat fmt = {}) {
        if constexpr (!available) {
            (void)fmt;
            return false;
        } else {
            return Resource::init({.route = route, .mode = UsartMode::sync,
                                   .bits = fmt.bits, .parity = fmt.parity,
                                   .two_stop = fmt.two_stop, .baud = 64,
                                   .sync_client = true});
        }
    }
    /// The fastest XCK this CLK_PER can recover (strictly below).
    static constexpr uint32_t max_xck_hz(uint32_t clk_per_hz) {
        return Resource::max_client_xck_hz(clk_per_hz);
    }
    static constexpr UsartPin clock_pin() { return usart_pin(n, route, UsartSignal::xck); }
};

/**
 * MspiHost<n, route>: the USART as an SPI host (27.3.3.1.3). Eight data
 * bits, no start/stop/parity, MSb first unless `lsb_first`; the
 * receiver's error flags read as zero in this mode. There is no client
 * select - the app drives one with a Pin - and no write-collision
 * protection or double speed. SCK = XCK, MOSI = TXD, MISO = RXD.
 */
template <uint8_t n, UsartRoute route = UsartRoute::def>
struct MspiHost : UsartTaskBase<n, route, UsartMode::mspi> {
    using Base = UsartTaskBase<n, route, UsartMode::mspi>;
    using Resource = Usart<n>;
    static constexpr bool available = Base::available;

    struct Options {
        bool lsb_first = false;      ///< UDORD
        bool sample_trailing = false;///< UCPHA: sample on the trailing edge
        bool invert_sck = false;     ///< the pin's INVEN: the other clock polarity
    };

    template <typename Clock>
    static bool init(Clock clock, uint32_t sck_hz, Options o = {}) {
        static_assert(clock_follows<Clock, MspiHost>(),
                      "MspiHost on a DynamicClock that does not list it: its SCK would go stale");
        if constexpr (!available) {
            (void)clock; (void)sck_hz; (void)o;
            return false;
        } else {
            const uint32_t hz = clock_hz(clock);
            if (sck_hz > Resource::max_host_xck_hz(hz)) return false;
            const uint16_t r = usart_baud_reg(hz, sck_hz, 2);
            if (r == 0) return false;
            Base::baud_ = sck_hz;
            if (!Resource::init({.route = route, .mode = UsartMode::mspi, .baud = r,
                                 .lsb_first = o.lsb_first,
                                 .sample_trailing = o.sample_trailing})) {
                return false;
            }
            constexpr UsartPin x = usart_pin(n, route, UsartSignal::xck);
            volatile uint8_t& c = pinctrl_of(x.port, x.pin);
            if (o.invert_sck) c |= PORT_INVEN_bm;
            else c &= static_cast<uint8_t>(~PORT_INVEN_bm);
            return true;
        }
    }
    static void rebase(uint32_t hz) {
        if (Base::baud_ == 0) return;
        const uint16_t r = usart_baud_reg(hz, Base::baud_, 2);
        if (r != 0) Resource::baud_reg(r);
    }

    /// One byte out, one byte in - the SPI exchange.
    static std::optional<uint8_t> transfer(uint8_t v, uint32_t spins = 500'000u) {
        if (!Resource::send(v, spins)) return {};
        const auto f = Resource::wait(spins);
        if (!f) return {};
        return static_cast<uint8_t>(f->data);
    }
};

/**
 * IrdaLink<n, route>: IRCOM (27.3.3.2.7), IrDA 1.4 up to 115.2 kbps.
 * The TXD/RXD levels are the inverse of the infrared pulses. TXPL picks
 * the transmit pulse shape (0 = 3/16 of a bit, 1..0xFE = that many
 * CLK_PER, 0xFF = coding off, which turns IRCOM into a pass-through);
 * RXPL is the receiver's minimum pulse width in samples. Double speed
 * is not available in this mode. receive_on() takes the receiver's
 * input from an event channel instead of the RXD pin.
 */
template <uint8_t n, UsartRoute route = UsartRoute::def>
struct IrdaLink : UsartTaskBase<n, route, UsartMode::ircom> {
    using Base = UsartTaskBase<n, route, UsartMode::ircom>;
    using Resource = Usart<n>;
    static constexpr bool available = usart_route_exists(n, route);

    /// 27.3.3.2.7: IrDA compatibility stops here.
    static constexpr uint32_t max_baud = 115'200;

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, uint8_t tx_pulse = 0, uint8_t rx_pulse = 0,
                     UsartFormat fmt = {}) {
        static_assert(clock_follows<Clock, IrdaLink>(),
                      "IrdaLink on a DynamicClock that does not list it: its baud would go stale");
        if constexpr (!available) {
            (void)clock; (void)baud; (void)tx_pulse; (void)rx_pulse; (void)fmt;
            return false;
        } else {
            if (baud > max_baud) return false;
            const uint16_t r = usart_baud_reg(clock_hz(clock), baud);
            if (r == 0) return false;
            Base::baud_ = baud;
            return Resource::init({.route = route, .mode = UsartMode::ircom,
                                   .bits = fmt.bits, .parity = fmt.parity,
                                   .two_stop = fmt.two_stop, .baud = r,
                                   .tx_pulse = tx_pulse, .rx_pulse = rx_pulse});
        }
    }

    /// Feed the IRCOM receiver from an event channel (EVCTRL.IREI);
    /// this disconnects the RXD pin.
    template <uint8_t ch>
    static void receive_on(EventChannel<ch> c) { Resource::irda_on(c); }
};

/**
 * AutoBaud<n, route>: the receiver measures the sender's rate from a
 * break field followed by a 0x55 sync field and writes BAUD itself
 * (27.3.3.2.5). GENAUTO accepts a break of any length once
 * wait_for_break() is called; LINAUTO wants a real LIN break (12+ low
 * peripheral clock cycles) and insists the sync character be 0x55.
 *
 * Errata 2.16.3 shapes the whole task: a sync field the hardware cannot
 * use sets ISFIF, and from that moment the receiver is dead - clearing
 * the flag is not enough, RXEN must be written 0 then 1. So every
 * receive here goes through recover(), and sync_error() is meant to be
 * polled (or ABEIE bound to the RXC vector).
 */
template <uint8_t n, UsartRoute route = UsartRoute::def>
struct AutoBaud : UsartTaskBase<n, route, UsartMode::async> {
    using Base = UsartTaskBase<n, route, UsartMode::async>;
    using Resource = Usart<n>;
    static constexpr bool available = usart_route_exists(n, route);

    enum class Kind : uint8_t { generic, lin };

    /// `fallback_baud` is what BAUD holds until a sync field replaces
    /// it - the rate the transmitter keeps using, since auto-baud only
    /// concerns the receiver.
    template <typename Clock>
    static bool init(Clock clock, uint32_t fallback_baud, Kind k = Kind::generic,
                     UsartAbWindow window = UsartAbWindow::wdw0, UsartFormat fmt = {}) {
        static_assert(clock_follows<Clock, AutoBaud>(),
                      "AutoBaud on a DynamicClock that does not list it: its fallback baud "
                      "would go stale");
        if constexpr (!available) {
            (void)clock; (void)fallback_baud; (void)k; (void)window; (void)fmt;
            return false;
        } else {
            const uint16_t r = usart_baud_reg(clock_hz(clock), fallback_baud);
            if (r == 0) return false;
            Base::baud_ = fallback_baud;
            return Resource::init({.route = route, .bits = fmt.bits, .parity = fmt.parity,
                                   .two_stop = fmt.two_stop,
                                   .rx_mode = k == Kind::lin ? UsartRxMode::linauto
                                                             : UsartRxMode::genauto,
                                   .baud = r, .ab_window = window});
        }
    }

    /// GENAUTO only: the next frame is treated as a break whatever its
    /// length. The bit disarms itself after that frame, and it is
    /// write-only - nothing can read back whether an arming stands.
    static void arm_break() { Resource::wait_for_break(); }
    /// A valid break plus sync character was seen; cleared by the next
    /// data frame or by hand.
    static bool break_detected() { return Resource::break_flag(); }
    static void clear_break() { Resource::clear_break(); }

    /// The sync field did not give a usable rate (or, in LINAUTO, was
    /// not 0x55). The receiver is dead until recover().
    static bool sync_error() { return Resource::isf_flag(); }
    /// Errata 2.16.3's work around: clear ISFIF and toggle RXEN.
    static void recover() { Resource::recover_from_isf(); }

    /// What the sync field measured, as a BAUD register value and as a
    /// rate at this peripheral clock.
    static uint16_t measured_baud_reg() { return Resource::baud_reg(); }
    static uint32_t measured_baud(uint32_t clk_per_hz) {
        return usart_actual_baud(clk_per_hz, Resource::baud_reg());
    }

    /// A frame, with the receiver revived first if a sync field failed.
    static std::optional<UsartFrame> poll() {
        if (Resource::isf_flag()) {
            recover();
            return {};
        }
        return Resource::poll();
    }
    /// The rebase of an auto-baud receiver only restores the fallback:
    /// a rate measured from the wire is not the driver's to recompute.
    static void rebase(uint32_t hz) {
        if (Base::baud_ == 0) return;
        (void)Resource::set_baud(hz, Base::baud_);
    }
};

static_assert(ClockUser<OneWire<0>>);
static_assert(ClockUser<Rs485<0>>);
static_assert(ClockUser<SyncHost<0>>);
static_assert(ClockUser<MspiHost<0>>);
static_assert(ClockUser<IrdaLink<0>>);
static_assert(ClockUser<AutoBaud<0>>);

} // namespace brio
