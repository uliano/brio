/*
 * spi.hpp
 *
 * The AVR DA/DB Serial Peripheral Interface (DS40002247B ch. 28) in two
 * strata, as docs/avrdx/spi.md describes:
 *
 *  RESOURCE - Spi<n>: the typed view of one instance. A config struct
 *  owns the whole configuration (route, host/client role, transfer
 *  mode, data order, bit rate, the Client Select policy, the buffer
 *  bits); init<cfg>() folds it and refuses at compile time what this
 *  package cannot bond, init(cfg) computes it at run time and returns
 *  false instead. Then the verbs the roles share: enable/disable, the
 *  role and its HOST DEMOTION readback, the rate, the mode, the data
 *  register, the two INTFLAGS layouts with the clear discipline of
 *  each flag, the interrupt enables, the two ISR bodies, and release() -
 *  route back to NONE with every pin handed back to PORT.
 *
 *  TASKS - what an application names:
 *    SpiHost<n, route>    the transfer ENGINE: two-phase descriptor
 *                         transactions in one chip-select window, the
 *                         instance in BUFFER MODE with two bytes in
 *                         flight on both completion styles - a polled
 *                         loop or the RXCIE pump -, the received bytes
 *                         READ BY COUNT, CS and DC owned by the engine.
 *                         Driven by util/spi_bus.hpp (arbitration and
 *                         replies).
 *    SpiClient<n, route>  the client side: the RUNS answer(), exchange()
 *                         and receive() - a transaction in one polled
 *                         call, in buffer mode read by the count the
 *                         client's transmitter flags keep of the host's
 *                         transfers -, the per-byte verbs, the ISR
 *                         bodies.
 *  A task owns its instance; two tasks on one Spi<n> is the app's bug.
 *
 * Facts that shape the code (28.3, 28.5, errata DS80000915F 2.11.1 +
 * clarifications 3.5.1/3.5.2/3.7.3, and DS80000882C 2.10.1):
 *  - SPI0 and SPI1 exist on EVERY package of the family (28, 32, 48 and
 *    64 pins alike): there are no instance tiers here, only route ones;
 *  - the routes are PORTMUX.SPIROUTEA, two bits per instance, with a
 *    NONE code that disconnects every pin. Which positions a package
 *    bonds is the device header's route enums, and they are the table
 *    below - 28/32-pin parts list DEFAULT and NONE only for both
 *    instances;
 *  - ERRATA BEATS HEADER: the 48-pin header lists SPI1 ALT2 (MOSI PB4,
 *    MISO PB5, no SCK, no SS position), and DB errata 2.11.1 says that
 *    position is NON-FUNCTIONAL on 48-pin devices (rev. A4/A5; fixed in
 *    B0). The DA errata (DS80000882C) has no twin item, but the 48-pin
 *    DA header bonds no SCK for that route either - a host cannot clock
 *    and a client cannot be clocked - so the route is refused on 48-pin
 *    parts of BOTH families, at compile time and at run time;
 *  - DA errata 2.10.1: with PORTMUX.SPIROUTE at NONE the Client Select
 *    line must be DISABLED (CTRLB.SSD = 1) for Host mode to survive. It
 *    is listed for every DA revision and not at all for the DB, but the
 *    configuration it forbids (a pinless host still watching an SS
 *    input that no pin can hold high) has no use, so it is refused on
 *    both families;
 *  - the seven rates: PRESC divides CLK_PER by 4/16/64/128 and CLK2X
 *    halves that (2/8/32/64). Div-64 is reachable twice (PRESC DIV64
 *    alone, PRESC DIV128 doubled); SpiClock names the seven distinct
 *    divisions and spi_presc_bits() picks the canonical encoding;
 *  - the host's SCK ceiling is CLK_PER/2 and a client's is CLK_PER/6
 *    (errata clarification 3.7.3 - the chapter's own "two peripheral
 *    clocks per SCK phase" would say /4; the tighter table wins);
 *  - TWO INTFLAGS LAYOUTS at the same address, chosen by CTRLB.BUFEN,
 *    with DIFFERENT clear disciplines (28.5.4, 28.5.5):
 *      normal mode: IF is cleared by writing a one to it OR by reading
 *      INTFLAGS while it is set and then accessing DATA; WRCOL has ONLY
 *      the read-then-DATA sequence documented, so clear_write_collision
 *      performs it;
 *      buffer mode: TXCIF, SSIF and BUFOVF are write-one-to-clear (a
 *      PLAIN store of the single bit - an RMW would write back every
 *      flag it read and clear the lot, the pin.hpp discipline); RXCIF
 *      and BUFOVF also clear by reading DATA, and DREIF is cleared by
 *      WRITING DATA - never by a store to INTFLAGS;
 *  - host demotion (28.3.2.1.3): with SSD = 0 and the SS pin an input
 *    driven low, the hardware clears CTRLA.MASTER, the instance becomes
 *    a client, and IF (normal) or SSIF (buffer) is set. Nothing but the
 *    application re-arms it: is_host() is the readback and
 *    restore_host() the re-arm;
 *  - in client mode SS, MOSI and SCK are always inputs and MISO is
 *    driven only while SS is low - the pad tri-states itself when the
 *    client is deselected (28.3.2.2.3), so the driver just sets the
 *    direction and lets the hardware gate it;
 *  - the SPI has one interrupt vector (SPIn_INT) for both layouts and
 *    one event generator (SCK level in host mode, evsys.hpp EvSpiSck);
 *    it has no event users and no DBGCTRL/RUNSTDBY of its own;
 *  - A READ THAT EMPTIES THE FIFO AS A BYTE ENTERS IT IS NOT TAKEN
 *    (measured, no erratum names it): the read returns the byte at the
 *    head and leaves it there, the arriving byte queues behind it, and
 *    RXCIF falls; the next read returns the SAME byte again and brings
 *    the flag back. A read that leaves a byte behind never meets that
 *    cycle, and neither does one made with nothing in flight. RXCIF is
 *    therefore a hint and never a count: SpiHost reads what the
 *    transmitter's flags prove has arrived, never empties the FIFO by a
 *    polled read while a byte shifts, and in the pump, which may, drops
 *    the repeated byte where the flags show the read was not taken;
 *    SpiClient's runs count the host's transfers on the client's DREIF
 *    and read the byte before the newest, for the same reason.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <span>
#include <type_traits>
#include <avr/io.h>

#include "avrdx/delay.hpp"
#include "avrdx/device_tables.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "avrdx/platform.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// ---- routes and pins (DS40002247B 17.5.8, 28.2.2; DS40002443A 17.3.5) -------

/// SPI pin routing (PORTMUX.SPIROUTEA), by NAME. On the AVR DA/DB the
/// values ARE the device header's group values, `none` included (DEFAULT
/// 0, ALT1 1, ALT2 2, NONE 3). On the AVR EA's layout SPI0's field is
/// three bits wide, code 2 is reserved and NONE is 7 (DS40002443A 17.3.5):
/// there the names ALT3..ALT6 exist, ALT2 does not, and every route's code
/// is the device tables' (spi_route_codes).
enum class SpiRoute : uint8_t {
    def = 0,
    alt1 = 1,
#if PORTMUX_SPI0_gm == 0x07   // the AVR EA's layout (device tables' portmux_layout)
    none = 3,
    alt3 = 4,
    alt4 = 5,
    alt5 = 6,
    alt6 = 7,
#else
    alt2 = 2,
    none = 3,
#endif
};
static_assert(portmux_layout != PortmuxLayout::dx ||
              (static_cast<uint8_t>(SpiRoute::def) == PORTMUX_SPI0_DEFAULT_gv &&
               static_cast<uint8_t>(SpiRoute::none) == PORTMUX_SPI0_NONE_gv),
              "the route codes must be the device header's group values");

/// The code a route writes into instance n's field: the value itself on
/// the AVR DA/DB, the device tables' code on the AVR EA.
constexpr uint8_t spi_route_code(uint8_t n, SpiRoute r) {
    if constexpr (portmux_layout == PortmuxLayout::dx) {
        (void)n;
        return static_cast<uint8_t>(r);
    } else {
        return n < spi_count ? spi_route_codes[n][static_cast<uint8_t>(r)] : route_absent;
    }
}

/// The route a field code means for instance n (`none` for a code no
/// route of the instance carries).
constexpr SpiRoute spi_route_of_code(uint8_t n, uint8_t code) {
    if constexpr (portmux_layout == PortmuxLayout::dx) {
        (void)n;
        return static_cast<SpiRoute>(code);
    } else {
        for (uint8_t i = 0; i < 8; ++i) {
            if (spi_route_code(n, static_cast<SpiRoute>(i)) == code) return static_cast<SpiRoute>(i);
        }
        return SpiRoute::none;
    }
}

// The pin count of this package is device_tables.hpp's package_pins:
// only its >= 48 and >= 64 steps change an SPI route table.

/// The four signals of a route (28.2.2), in pin order.
enum class SpiSignal : uint8_t { mosi = 0, miso = 1, sck = 2, ss = 3 };

/// Where one signal of one route sits, and whether THIS package bonds
/// it out. `bonded == false` means the silicon has the function but no
/// pin for it here.
struct SpiPin {
    char port = '?';
    uint8_t pin = 0;
    bool bonded = false;
    constexpr explicit operator bool() const { return bonded; }
};

#if PORTMUX_SPI0_gm == 0x07
// ---- the AVR EA's route table ------------------------------------------------
// SPI0 alone (DS40002443A 17.3.5, table 3-1):
//   DEFAULT PA4-PA7   ALT1 PE0-PE3 (48 pins)   ALT4 PD4-PD7   ALT5 PC0-PC3
//   ALT3 MOSI PA0, MISO PA1, SCK PC0, SS PC1   (two ports)
//   ALT6 MOSI PC1, MISO PC2, SCK PC3, SS PF7   (two ports; SS on the UPDI pin)

/// The routes whose SS position is PF7, the UPDI pin (ALT6). PF7 is a GPIO
/// only with FUSE.SYSCFG0.UPDIPINCFG = 0, which leaves the part
/// unreachable without a high-voltage programmer: refused at compile time
/// rather than offered as a pin that costs the debug link.
constexpr bool spi_route_on_updi_pin(uint8_t n, SpiRoute r) { return n == 0 && r == SpiRoute::alt6; }

/// The routes whose four signals do not sit on one port (ALT3 and ALT6).
/// The pin set-up below hands over one port per route; such a route is
/// refused until the engine is written for it (docs/avrdx/spi.md).
constexpr bool spi_route_crosses_ports(uint8_t n, SpiRoute r) {
    return n == 0 && (r == SpiRoute::alt3 || r == SpiRoute::alt6);
}

constexpr char spi_port_letter(uint8_t, SpiRoute r) {
    switch (r) {
        case SpiRoute::def: return 'A';
        case SpiRoute::alt1: return 'E';
        case SpiRoute::alt4: return 'D';
        case SpiRoute::alt5: return 'C';
        default: return '?';
    }
}

constexpr uint8_t spi_first_pin(uint8_t, SpiRoute r) {
    return r == SpiRoute::def || r == SpiRoute::alt4 ? 4 : 0;
}

/// Whether this package offers a route at all. NONE always exists (an
/// instance runs pinless). ALT1 needs the 48-pin part's PORTE; ALT3 and
/// ALT6 exist in the silicon and are refused by name at the tasks.
constexpr bool spi_route_exists(uint8_t n, SpiRoute r) {
    if (n >= spi_count) return false;
    if (spi_route_code(n, r) == route_absent) return false;
    if (r == SpiRoute::alt1) return package_pins >= 48;
    return true;
}

/// The position of one signal of one route on THIS package.
constexpr SpiPin spi_pin(uint8_t n, SpiRoute r, SpiSignal s) {
    if (n >= spi_count || r == SpiRoute::none || !spi_route_exists(n, r)) return {};
    const char port = spi_port_letter(n, r);
    if (port == '?' || !port_exists(port)) return {};
    return {port, static_cast<uint8_t>(spi_first_pin(n, r) + static_cast<uint8_t>(s)), true};
}

#else
// ---- the AVR DA/DB's route table ---------------------------------------------

/// No route of this layout puts SS on the UPDI pin, and none crosses ports.
constexpr bool spi_route_on_updi_pin(uint8_t, SpiRoute) { return false; }
constexpr bool spi_route_crosses_ports(uint8_t, SpiRoute) { return false; }

/// The port and first pin of a route, straight from the device headers:
///   SPI0 DEFAULT PA4-PA7   ALT1 PE0-PE3 (48+)   ALT2 PG4-PG7 (64)
///   SPI1 DEFAULT PC0-PC3   ALT1 PC4-PC7 (48+)   ALT2 PB4-PB7 (64;
///                                               48-pin bonds PB4/PB5
///                                               only and is refused)
constexpr char spi_port_letter(uint8_t n, SpiRoute r) {
    if (n == 0) {
        switch (r) {
            case SpiRoute::def: return 'A';
            case SpiRoute::alt1: return 'E';
            case SpiRoute::alt2: return 'G';
            default: return '?';
        }
    }
    switch (r) {
        case SpiRoute::def: return 'C';
        case SpiRoute::alt1: return 'C';
        case SpiRoute::alt2: return 'B';
        default: return '?';
    }
}

constexpr uint8_t spi_first_pin(uint8_t n, SpiRoute r) {
    if (n == 0) return r == SpiRoute::def ? 4 : (r == SpiRoute::alt1 ? 0 : 4);
    return r == SpiRoute::def ? 0 : 4;
}

/// Whether this package offers a route at all. NONE always exists (an
/// instance runs pinless: the shift register, the flags and, in host
/// mode, the SCK event generator are all live). The rest is the device
/// header's route enum, plus the erratum that beats it.
constexpr bool spi_route_exists(uint8_t n, SpiRoute r) {
    if (n >= spi_count) return false;
    if (r == SpiRoute::none) return true;
    if (r == SpiRoute::def) return true;                  // both instances, every package
    if (r == SpiRoute::alt1) return package_pins >= 48;
    // ALT2: SPI0 on PORTG (64-pin only); SPI1 on PORTB - listed by the
    // 48-pin headers without SCK/SS and declared non-functional there
    // by DB errata 2.11.1, so 64 pins for both.
    if (package_pins < 64) return false;
    return port_exists(spi_port_letter(n, r));
}

/// The position of one signal of one route on THIS package.
constexpr SpiPin spi_pin(uint8_t n, SpiRoute r, SpiSignal s) {
    if (n >= spi_count || r == SpiRoute::none) return {};
    const char port = spi_port_letter(n, r);
    if (port == '?' || !port_exists(port)) return {};
    const uint8_t pin = static_cast<uint8_t>(spi_first_pin(n, r) + static_cast<uint8_t>(s));
    // SPI1 ALT2 on a 48-pin package stops at PB5: the route as a whole
    // is refused above, but the table stays honest about why.
    if (n == 1 && r == SpiRoute::alt2 && package_pins < 64 &&
        static_cast<uint8_t>(s) >= 2) {
        return {port, pin, false};
    }
    return {port, pin, true};
}
#endif

// ---- the knobs (28.5.1 - 28.5.2) --------------------------------------------

/// Which end of the bus this instance is (CTRLA.MASTER).
enum class SpiRole : uint8_t { host, client };

/// CTRLB.MODE: the four clock phase/polarity combinations (28.3.2.3).
/// The values ARE the device header's codes.
enum class SpiMode : uint8_t {
    mode0 = SPI_MODE_0_gc,   ///< SCK idle low,  sample on the leading (rising) edge
    mode1 = SPI_MODE_1_gc,   ///< SCK idle low,  sample on the trailing (falling) edge
    mode2 = SPI_MODE_2_gc,   ///< SCK idle high, sample on the leading (falling) edge
    mode3 = SPI_MODE_3_gc,   ///< SCK idle high, sample on the trailing (rising) edge
};

/// The idle level of SCK for a mode (MODE bit 1 = CPOL): modes 2 and 3
/// leave the line high between transfers.
constexpr bool spi_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 0x02u) != 0; }
/// CPHA (MODE bit 0): the sampling edge is the trailing one.
constexpr bool spi_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 0x01u) != 0; }

/// The SEVEN host bit rates, named by what they divide CLK_PER by
/// (28.5.1: PRESC picks 4/16/64/128, CLK2X halves it). CLK_PER/64 is
/// reachable two ways; the canonical encoding here is PRESC DIV64
/// without the doubler.
enum class SpiClock : uint8_t {
    div2 = 0,
    div4 = 1,
    div8 = 2,
    div16 = 3,
    div32 = 4,
    div64 = 5,
    div128 = 6,
};

/// What this rate divides CLK_PER by.
constexpr uint8_t spi_division(SpiClock c) {
    switch (c) {
        case SpiClock::div2: return 2;
        case SpiClock::div4: return 4;
        case SpiClock::div8: return 8;
        case SpiClock::div16: return 16;
        case SpiClock::div32: return 32;
        case SpiClock::div64: return 64;
        default: return 128;
    }
}

/// The CTRLA bits (PRESC and CLK2X) that produce this rate. The enum's
/// order IS the register's: its two high bits are PRESC (DIV4 for div2
/// and div4, DIV16 for div8 and div16, DIV64 for div32 and div64, DIV128
/// for div128) and CLK2X belongs to the even member of each pair -
/// div128 excepted, whose doubled twin would be div64 again. Arithmetic
/// and no table, so a host's fold of a request's rate into its register
/// byte is a handful of instructions in line.
constexpr uint8_t spi_presc_bits(SpiClock c) {
    const uint8_t v = static_cast<uint8_t>(c);
    const uint8_t presc = static_cast<uint8_t>(v & 0x06u);
    const bool doubled = (v & 0x01u) == 0 && c != SpiClock::div128;
    return static_cast<uint8_t>(presc | (doubled ? SPI_CLK2X_bm : 0));
}
static_assert(SPI_PRESC_DIV4_gc == 0x00 && SPI_PRESC_DIV16_gc == 0x02 &&
              SPI_PRESC_DIV64_gc == 0x04 && SPI_PRESC_DIV128_gc == 0x06,
              "spi_presc_bits reads PRESC off the enum's bits: the device header's codes");
static_assert(spi_presc_bits(SpiClock::div2) == (SPI_PRESC_DIV4_gc | SPI_CLK2X_bm) &&
              spi_presc_bits(SpiClock::div16) == SPI_PRESC_DIV16_gc &&
              spi_presc_bits(SpiClock::div32) == (SPI_PRESC_DIV64_gc | SPI_CLK2X_bm) &&
              spi_presc_bits(SpiClock::div128) == SPI_PRESC_DIV128_gc);

/// The rate the CTRLA bits in force encode (the readback of the line
/// above; PRESC DIV128 with CLK2X reads back as div64, its twin).
constexpr SpiClock spi_clock_of(uint8_t ctrla) {
    const uint8_t presc = static_cast<uint8_t>(ctrla & SPI_PRESC_gm);
    const bool x2 = (ctrla & SPI_CLK2X_bm) != 0;
    if (presc == SPI_PRESC_DIV4_gc) return x2 ? SpiClock::div2 : SpiClock::div4;
    if (presc == SPI_PRESC_DIV16_gc) return x2 ? SpiClock::div8 : SpiClock::div16;
    if (presc == SPI_PRESC_DIV64_gc) return x2 ? SpiClock::div32 : SpiClock::div64;
    return x2 ? SpiClock::div64 : SpiClock::div128;
}

/// The SCK frequency a rate produces at a peripheral clock.
constexpr uint32_t spi_sck_hz(uint32_t clk_per_hz, SpiClock c) {
    return clk_per_hz / spi_division(c);
}

/// The FASTEST rate whose SCK does not exceed `max_sck_hz` - the
/// chooser a device's datasheet limit is spoken to. Empty when even
/// CLK_PER/128 is too fast for it.
constexpr std::optional<SpiClock> spi_clock_for(uint32_t clk_per_hz, uint32_t max_sck_hz) {
    if (max_sck_hz == 0) return {};
    for (uint8_t i = 0; i <= static_cast<uint8_t>(SpiClock::div128); ++i) {
        const SpiClock c = static_cast<SpiClock>(i);
        if (spi_sck_hz(clk_per_hz, c) <= max_sck_hz) return c;
    }
    return {};
}

/// The ceilings of the two roles (errata clarification 3.7.3): a host
/// divides CLK_PER by at least two; a client's incoming SCK must stay
/// at or below CLK_PER/6.
constexpr uint32_t spi_max_host_sck_hz(uint32_t clk_per_hz) { return clk_per_hz / 2; }
constexpr uint32_t spi_max_client_sck_hz(uint32_t clk_per_hz) { return clk_per_hz / 6; }

/// Everything one instance is configured with.
struct SpiConfig {
    SpiRoute route = SpiRoute::def;
    SpiRole role = SpiRole::host;
    SpiMode mode = SpiMode::mode0;
    SpiClock clock = SpiClock::div16;   ///< host only (PRESC/CLK2X are ignored by a client)
    bool lsb_first = false;             ///< DORD
    bool client_select_disable = true;  ///< SSD: the host ignores the SS pin (no multi-host)
    bool ss_output = false;             ///< host, SSD = 0: drive SS out (a host that keeps itself selected)
    bool buffer_mode = false;           ///< BUFEN: two TX levels, a two-deep RX FIFO, four flags
    bool buffer_wait = false;           ///< BUFWR: client only - the first write goes straight to the shifter
    bool drive_miso = true;             ///< client: MISO an output (the hardware tri-states it while SS is high)
    bool enable = true;                 ///< leave the instance enabled when init() returns
};

/// Is this configuration legal on this package?
///  - the route must exist here (the erratum included);
///  - a host on the pinless route must have SSD set (DA errata 2.10.1);
///  - a host that drives SS out has to have an SS pin to drive, and so
///    does a host that watches one (SSD = 0);
///  - a client needs SCK, MOSI and SS pins: there is nothing for a
///    pinless client to be clocked by.
template <uint8_t n>
constexpr bool spi_config_valid(const SpiConfig& c) {
    if (n >= spi_count) return false;
    if (!spi_route_exists(n, c.route)) return false;
    if (spi_route_on_updi_pin(n, c.route) || spi_route_crosses_ports(n, c.route)) return false;
    const bool pinless = c.route == SpiRoute::none;
    if (c.role == SpiRole::host) {
        if (pinless) return c.client_select_disable && !c.ss_output;
        if (!c.client_select_disable && !spi_pin(n, c.route, SpiSignal::ss).bonded) return false;
        if (c.ss_output && (c.client_select_disable ||
                            !spi_pin(n, c.route, SpiSignal::ss).bonded)) return false;
        if (!spi_pin(n, c.route, SpiSignal::sck).bonded) return false;
        return true;
    }
    if (pinless) return false;
    if (c.buffer_wait && !c.buffer_mode) return false;   // BUFWR has no meaning without BUFEN
    return spi_pin(n, c.route, SpiSignal::sck).bonded &&
           spi_pin(n, c.route, SpiSignal::mosi).bonded &&
           spi_pin(n, c.route, SpiSignal::ss).bonded &&
           (!c.drive_miso || spi_pin(n, c.route, SpiSignal::miso).bonded);
}

// ---- the resource -----------------------------------------------------------

template <uint8_t n>
class Spi {
    static_assert(n < spi_count, "no such SPI on this part (the AVR DA/DB carry SPI0 and "
                                 "SPI1 on every package)");

public:
    Spi() = delete;

    static constexpr uint8_t index = n;

    /// The event vocabulary of this instance (evsys.hpp): the SCK level
    /// in host mode. The SPI has no event users.
    using SckEvent = EvSpiSck<n>;

    // ---- the route table of this instance --------------------------------

    static constexpr bool has_route(SpiRoute r) { return spi_route_exists(n, r); }
    static constexpr SpiPin mosi(SpiRoute r) { return spi_pin(n, r, SpiSignal::mosi); }
    static constexpr SpiPin miso(SpiRoute r) { return spi_pin(n, r, SpiSignal::miso); }
    static constexpr SpiPin sck(SpiRoute r) { return spi_pin(n, r, SpiSignal::sck); }
    static constexpr SpiPin ss(SpiRoute r) { return spi_pin(n, r, SpiSignal::ss); }

    /// The route in force since the last init()/release().
    static SpiRoute route() { return route_; }

    // ---- configuration ---------------------------------------------------

    /// Compile-time form: what this package cannot bond - and what the
    /// errata forbid - is refused here.
    template <SpiConfig cfg>
    static bool init() {
        static_assert(!spi_route_on_updi_pin(n, cfg.route),
                      "SPI0 ALT6 puts SS on PF7, the UPDI pin: a GPIO only with "
                      "FUSE.SYSCFG0.UPDIPINCFG = 0, which leaves the part unreachable "
                      "without a high-voltage programmer - refused");
        static_assert(!spi_route_crosses_ports(n, cfg.route),
                      "this route's signals sit on two ports (AVR EA SPI0 ALT3: PA0, PA1, "
                      "PC0, PC1), which the pin hand-over does not drive yet");
        static_assert(spi_route_exists(n, cfg.route),
                      "this part has not got this SPI route, or this package does not bond it "
                      "(AVR DA/DB: 28/32 pins have DEFAULT only, SPI1 ALT2 is refused on 48-pin "
                      "parts by DB errata 2.11.1; AVR EA: code 2 is reserved, ALT1 needs 48 pins)");
        static_assert(spi_config_valid<n>(cfg),
                      "this SPI configuration is not legal on this package: a pinless "
                      "host must disable Client Select (DA errata 2.10.1), a host needs "
                      "an SCK pin (and an SS pin when it watches or drives one), and a "
                      "client needs SCK, MOSI and SS");
        return init(cfg);
    }

    /// Run-time form. Stops the instance, releases the pins of the old
    /// route, routes and drives the new ones, writes the whole register
    /// set, then enables. False (and nothing programmed) when the
    /// config is not legal on this package.
    static bool init(const SpiConfig& cfg) {
        if (!spi_config_valid<n>(cfg)) return false;
        auto& s = regs();
        s.CTRLA = 0;                      // disabled: the pins go back to PORT control
        s.INTCTRL = 0;
        buffered_ = cfg.buffer_mode;
        mode_ = cfg.mode;
        role_ = cfg.role;
        setup_pins(cfg);
        s.CTRLB = ctrlb_byte(cfg);
        // The write-one-to-clear flags of BOTH layouts (IF/WRCOL are
        // RXCIF/TXCIF's bits), DREIF excepted: it is cleared by writing
        // DATA and reads as "transmitter ready".
        s.INTFLAGS = static_cast<uint8_t>(SPI_RXCIF_bm | SPI_TXCIF_bm |
                                          SPI_SSIF_bm | SPI_BUFOVF_bm);
        s.CTRLA = static_cast<uint8_t>(ctrla_byte(cfg) |
                                       (cfg.enable ? SPI_ENABLE_bm : 0));
        return true;
    }

    /// Stop the instance and hand its pins back: disabled, interrupts
    /// off, PORTMUX to NONE, every pin this driver drove returned to an
    /// input with its pull-up cleared.
    static void release() {
        auto& s = regs();
        s.CTRLA = 0;
        s.INTCTRL = 0;
        release_pins();
        write_route(SpiRoute::none);
        route_ = SpiRoute::none;
    }

    // ---- CTRLA -----------------------------------------------------------

    static void enable(bool on) { ctrla(SPI_ENABLE_bm, on); }
    static bool enabled() { return (regs().CTRLA & SPI_ENABLE_bm) != 0; }

    /// CTRLA.MASTER. The HARDWARE clears it on a demotion (28.3.2.1.3),
    /// so this is a readback, not a memory of what was asked for.
    static bool is_host() { return (regs().CTRLA & SPI_MASTER_bm) != 0; }
    /// The role init() asked for - what is_host() is compared against.
    static SpiRole role() { return role_; }
    /// True when this instance was configured as a host and the SS pin
    /// has since taken it out of Host mode.
    static bool demoted() { return role_ == SpiRole::host && !is_host(); }
    /// Re-arm Host mode after a demotion: the flag that reported it is
    /// cleared first (in normal mode by the read-then-DATA sequence,
    /// which is the only one that also clears WRCOL; in buffer mode
    /// SSIF is written one).
    static void restore_host() {
        if (buffered_) {
            clear_ss_flag();
        } else {
            clear_flags_by_data_access();
        }
        ctrla(SPI_MASTER_bm, true);
    }

    /// The host bit rate (PRESC + CLK2X). No effect in client mode.
    static void clock(SpiClock c) {
        regs().CTRLA = static_cast<uint8_t>(
            (regs().CTRLA & ~(SPI_PRESC_gm | SPI_CLK2X_bm)) | spi_presc_bits(c));
    }
    static SpiClock clock() { return spi_clock_of(regs().CTRLA); }
    /// The SCK this instance produces at a peripheral clock.
    static uint32_t sck_hz(uint32_t clk_per_hz) { return spi_sck_hz(clk_per_hz, clock()); }

    /// DORD: LSb first when set.
    static void lsb_first(bool on) { ctrla(SPI_DORD_bm, on); }
    static bool lsb_first() { return (regs().CTRLA & SPI_DORD_bm) != 0; }

    // ---- CTRLB -----------------------------------------------------------

    /// The transfer mode. A CPOL change moves the SCK line's idle level,
    /// and the AVR only refreshes the SCK OUTPUT while the peripheral is
    /// enabled and at a transfer - not on a CTRLB write (bench: SCK
    /// still low 10 us after a switch to mode 3). SpiHost::apply_mode
    /// does the disable/preset/enable dance around this verb; a caller
    /// changing polarity by hand must do the same.
    static void mode(SpiMode m) {
        mode_ = m;
        regs().CTRLB = static_cast<uint8_t>((regs().CTRLB & ~SPI_MODE_gm) |
                                            static_cast<uint8_t>(m));
    }
    static SpiMode mode() { return static_cast<SpiMode>(regs().CTRLB & SPI_MODE_gm); }

    /// SSD: a host with this bit set ignores the SS pin entirely (no
    /// multi-host demotion, the pin free for anything else).
    static void client_select_disable(bool on) { ctrlb(SPI_SSD_bm, on); }
    static bool client_select_disabled() { return (regs().CTRLB & SPI_SSD_bm) != 0; }

    /// BUFEN (and BUFWR, which only a client honours). Switching the
    /// layout also switches which INTFLAGS bits this driver reports.
    static void buffer_mode(bool on, bool wait_for_receive = false) {
        buffered_ = on;
        regs().CTRLB = static_cast<uint8_t>((regs().CTRLB & ~(SPI_BUFEN_bm | SPI_BUFWR_bm)) |
                                            (on ? SPI_BUFEN_bm : 0) |
                                            (wait_for_receive ? SPI_BUFWR_bm : 0));
    }
    static bool buffer_mode() { return (regs().CTRLB & SPI_BUFEN_bm) != 0; }
    static bool buffer_wait() { return (regs().CTRLB & SPI_BUFWR_bm) != 0; }

    // ---- data (28.5.6) ---------------------------------------------------

    /// Write DATA: in host mode this STARTS a transfer; in client mode
    /// it loads what the next incoming clock will shift out. In normal
    /// mode a write while a transfer is running is ignored and sets
    /// WRCOL; in buffer mode it is accepted while DREIF is set.
    static void write(uint8_t v) { regs().DATA = v; }
    /// Read DATA. This access is also half of the normal-mode clear
    /// sequence and the way RXCIF/BUFOVF clear in buffer mode.
    static uint8_t read() { return regs().DATA; }

    /// One byte if the receiver has one (IF in normal mode, RXCIF in
    /// buffer mode).
    static std::optional<uint8_t> poll() {
        const uint8_t f = regs().INTFLAGS;
        if (buffered_) {
            if ((f & SPI_RXCIF_bm) == 0) return {};
        } else if ((f & SPI_IF_bm) == 0) {
            return {};
        }
        return regs().DATA;
    }

    /// Wait (bounded) for a byte.
    static std::optional<uint8_t> wait(uint32_t spins = 500'000u) {
        for (;;) {
            if (const auto v = poll()) return v;
            if (spins-- == 0) return {};
        }
    }

    /// One polled HOST byte: write, spin on the completion flag, read
    /// back. The spin is one byte time by construction (32 CPU cycles
    /// at CLK_PER/4); `spins` bounds a bus that never answers, e.g. a
    /// host that has been demoted mid-transfer.
    static std::optional<uint8_t> transfer(uint8_t out, uint32_t spins = 500'000u) {
        regs().DATA = out;
        return wait(spins);
    }

    // ---- flags: normal mode (28.5.4) -------------------------------------

    static uint8_t flags() { return regs().INTFLAGS; }

    /// IF: a byte has been shifted through - or, in a host with SSD = 0,
    /// the SS pin has just demoted it.
    static bool if_flag() { return (regs().INTFLAGS & SPI_IF_bm) != 0; }
    /// WRCOL: DATA was written while a transfer was running (the write
    /// was ignored; the transfer itself is unharmed).
    static bool write_collision() { return (regs().INTFLAGS & SPI_WRCOL_bm) != 0; }
    /// The documented W1C half of the normal layout: IF alone (a PLAIN
    /// store of the one bit - the pin.hpp discipline). WRCOL has no W1C
    /// path in the register description: use clear_flags_by_data_access.
    static void clear_if() { regs().INTFLAGS = SPI_IF_bm; }
    /// The other documented sequence: read INTFLAGS while the flag is
    /// set, then access DATA. It is the ONLY way the chapter gives for
    /// WRCOL, and it clears IF with it.
    static void clear_flags_by_data_access() {
        (void)regs().INTFLAGS;
        (void)regs().DATA;
    }

    // ---- flags: buffer mode (28.5.5) -------------------------------------

    /// RXCIF: unread data in the receive buffer. Cleared by reading
    /// DATA until the FIFO is empty, or by writing a one here.
    static bool rxc_flag() { return (regs().INTFLAGS & SPI_RXCIF_bm) != 0; }
    /// TXCIF: the shifter and the transmit buffer are both empty. W1C.
    static bool txc_flag() { return (regs().INTFLAGS & SPI_TXCIF_bm) != 0; }
    /// DREIF: room in the transmit buffer. Cleared by WRITING DATA -
    /// never by a store to INTFLAGS.
    static bool dre_flag() { return (regs().INTFLAGS & SPI_DREIF_bm) != 0; }
    /// SSIF: the SS pin took a host out of Host mode (SSD = 0 only). W1C.
    static bool ss_flag() { return (regs().INTFLAGS & SPI_SSIF_bm) != 0; }
    /// BUFOVF: a third byte arrived with the two receive buffers full.
    /// Cleared by reading DATA or by writing a one here.
    static bool overflow_flag() { return (regs().INTFLAGS & SPI_BUFOVF_bm) != 0; }

    // Plain single-bit stores: an RMW would write back every flag it
    // read and clear the lot.
    static void clear_rxc() { regs().INTFLAGS = SPI_RXCIF_bm; }
    static void clear_txc() { regs().INTFLAGS = SPI_TXCIF_bm; }
    static void clear_ss_flag() { regs().INTFLAGS = SPI_SSIF_bm; }
    static void clear_overflow() { regs().INTFLAGS = SPI_BUFOVF_bm; }

    /// The receive FIFO emptied, in buffer mode with nothing shifting:
    /// one DATA read whatever RXCIF says - the read that takes a byte the
    /// flag no longer shows and brings the flag back -, then a read per
    /// entry the flag shows (the FIFO's two and the shifter's one at
    /// most), BUFOVF cleared. A read of an empty FIFO returns the last
    /// byte read and changes nothing after it (docs/avrdx/spi.md). init()
    /// does NOT do this: its store of one to RXCIF drops the flag and
    /// leaves the bytes, so a FIFO a former user left non-empty starts
    /// the next buffer-mode user with the flag down over it - and an
    /// RXCIF-paced reader is deaf from then on, every arrival queueing
    /// behind bytes no flag announces. SpiHost's init() and recover()
    /// call it, and SpiClient::init() in buffer mode.
    static void flush_receive() {
        auto& s = regs();
        (void)s.DATA;
        for (uint8_t i = 0; i < 3u && (s.INTFLAGS & SPI_RXCIF_bm) != 0; ++i) {
            (void)s.DATA;
        }
        s.INTFLAGS = SPI_BUFOVF_bm;
    }

    // ---- interrupts (28.5.3) ---------------------------------------------

    /// IE: the one enable of the normal layout (fires on IF).
    static void enable_interrupt(bool on) { intctrl(SPI_IE_bm, on); }
    /// The four buffer-mode enables. They are ignored outside buffer
    /// mode (28.5.3: "In the Non-Buffer mode, this bit is 0").
    static void enable_rxc_interrupt(bool on) { intctrl(SPI_RXCIE_bm, on); }
    static void enable_txc_interrupt(bool on) { intctrl(SPI_TXCIE_bm, on); }
    static void enable_dre_interrupt(bool on) { intctrl(SPI_DREIE_bm, on); }
    static void enable_ss_interrupt(bool on) { intctrl(SPI_SSIE_bm, on); }
    static uint8_t interrupts() { return regs().INTCTRL; }

    // ---- ISR bodies (one vector, SPIn_INT) --------------------------------

    /// What a normal-mode interrupt found: the flags as they were, and
    /// the byte (the read of DATA that clears IF and WRCOL is part of
    /// the documented sequence, so it happens here whether the caller
    /// wants the byte or not).
    struct NormalIsr {
        uint8_t flags;
        uint8_t data;
        constexpr bool complete() const { return (flags & SPI_IF_bm) != 0; }
        constexpr bool collision() const { return (flags & SPI_WRCOL_bm) != 0; }
    };

    /// ISR body for SPIn_INT_vect in NORMAL mode: read INTFLAGS, then
    /// DATA - the clear sequence - and hand both back.
    [[gnu::always_inline]] static NormalIsr take_normal() {
        const uint8_t f = regs().INTFLAGS;
        const uint8_t d = regs().DATA;
        return {f, d};
    }

    /// ISR body for SPIn_INT_vect in BUFFER mode: the flags, with the
    /// write-one-to-clear ones cleared (TXCIF, SSIF, BUFOVF by default).
    /// RXCIF and DREIF are NOT cleared here - they follow the data: the
    /// handler reads DATA to drain the FIFO and writes DATA to refill
    /// the transmitter.
    [[gnu::always_inline]] static uint8_t take_buffer(
        uint8_t clear = static_cast<uint8_t>(SPI_TXCIF_bm | SPI_SSIF_bm | SPI_BUFOVF_bm)) {
        const uint8_t f = regs().INTFLAGS;
        const uint8_t w = static_cast<uint8_t>(f & clear);
        if (w != 0) regs().INTFLAGS = w;
        return f;
    }

    // ---- registers and pins ----------------------------------------------

    static constexpr SPI_t& regs() {
#if defined(SPI1)
        if constexpr (n == 0) return SPI0;
        else return SPI1;
#else
        return SPI0;
#endif
    }

    /// PORTMUX position of this instance's route field (the device
    /// header's own group position, device_tables.hpp).
    static constexpr uint8_t route_gp() { return spi_route_gp(n); }

    /// Write this instance's route field: the route's CODE (the device
    /// tables' spi_route_codes) into the header's mask and position.
    static void write_route(SpiRoute r) {
        constexpr uint8_t gp = route_gp();
        constexpr uint8_t gm = spi_route_gm(n);
        if constexpr (portmux_layout == PortmuxLayout::dx) {
            PORTMUX.SPIROUTEA = static_cast<uint8_t>(
                (PORTMUX.SPIROUTEA & ~gm) | static_cast<uint8_t>(static_cast<uint8_t>(r) << gp));
        } else {
            PORTMUX.SPIROUTEA = static_cast<uint8_t>(
                (PORTMUX.SPIROUTEA & ~gm) | static_cast<uint8_t>(spi_route_code(n, r) << gp));
        }
    }

    /// The route the field reads back as (a code no route of this
    /// instance carries reads as `none`).
    static SpiRoute routed() {
        constexpr uint8_t gp = route_gp();
        constexpr uint8_t gm = spi_route_gm(n);
        if constexpr (portmux_layout == PortmuxLayout::dx) {
            return static_cast<SpiRoute>((PORTMUX.SPIROUTEA & gm) >> gp);
        } else {
            return spi_route_of_code(n, static_cast<uint8_t>((PORTMUX.SPIROUTEA & gm) >> gp));
        }
    }

private:
    static constexpr uint8_t ctrla_byte(const SpiConfig& c) {
        return static_cast<uint8_t>(
            (c.lsb_first ? SPI_DORD_bm : 0) |
            (c.role == SpiRole::host ? SPI_MASTER_bm : 0) |
            (c.role == SpiRole::host ? spi_presc_bits(c.clock) : 0));
    }

    static constexpr uint8_t ctrlb_byte(const SpiConfig& c) {
        return static_cast<uint8_t>(
            (c.buffer_mode ? SPI_BUFEN_bm : 0) |
            (c.buffer_wait ? SPI_BUFWR_bm : 0) |
            (c.client_select_disable ? SPI_SSD_bm : 0) |
            static_cast<uint8_t>(c.mode));
    }

    static void ctrla(uint8_t bit, bool on) {
        if (on) regs().CTRLA |= bit; else regs().CTRLA &= static_cast<uint8_t>(~bit);
    }
    static void ctrlb(uint8_t bit, bool on) {
        if (on) regs().CTRLB |= bit; else regs().CTRLB &= static_cast<uint8_t>(~bit);
    }
    static void intctrl(uint8_t bit, bool on) {
        if (on) regs().INTCTRL |= bit; else regs().INTCTRL &= static_cast<uint8_t>(~bit);
    }

    /// Route, then give PORT the directions the role needs. Runs off the
    /// route table through pin.hpp's run-time port lookup, so a position
    /// this package lacks never instantiates a Pin that does not exist.
    ///
    /// Host: MOSI and SCK outputs, SCK PRESET to the mode's idle level
    /// BEFORE it is driven (a device that latches its mode from SCK at
    /// the CS edge must never see the wrong polarity - the MCP3550
    /// does); MISO an input; SS driven high when asked, left an input
    /// with its PULL-UP on when the host watches it (a floating SS would
    /// demote the host at random), released when SSD is set.
    /// Client: MOSI, SCK and SS inputs; MISO an output when asked - the
    /// pad tri-states itself while SS is high (28.3.2.2.3).
    static void setup_pins(const SpiConfig& c) {
        release_pins();
        write_route(c.route);
        route_ = c.route;
        if (c.route == SpiRoute::none) return;
        const char p = spi_port_letter(n, c.route);
        volatile PORT_t& port = port_by_letter(p);
        const SpiPin mo = spi_pin(n, c.route, SpiSignal::mosi);
        const SpiPin mi = spi_pin(n, c.route, SpiSignal::miso);
        const SpiPin sc = spi_pin(n, c.route, SpiSignal::sck);
        const SpiPin sl = spi_pin(n, c.route, SpiSignal::ss);
        if (c.role == SpiRole::host) {
            if (sc.bonded) {
                if (spi_cpol(c.mode)) port.OUTSET = static_cast<uint8_t>(1u << sc.pin);
                else port.OUTCLR = static_cast<uint8_t>(1u << sc.pin);
                port.DIRSET = static_cast<uint8_t>(1u << sc.pin);
                driven_ = static_cast<uint8_t>(driven_ | (1u << sc.pin));
            }
            if (mo.bonded) {
                port.DIRSET = static_cast<uint8_t>(1u << mo.pin);
                driven_ = static_cast<uint8_t>(driven_ | (1u << mo.pin));
            }
            if (mi.bonded) port.DIRCLR = static_cast<uint8_t>(1u << mi.pin);
            if (sl.bonded && !c.client_select_disable) {
                if (c.ss_output) {
                    port.OUTSET = static_cast<uint8_t>(1u << sl.pin);   // high = still the host
                    port.DIRSET = static_cast<uint8_t>(1u << sl.pin);
                    driven_ = static_cast<uint8_t>(driven_ | (1u << sl.pin));
                } else {
                    port.DIRCLR = static_cast<uint8_t>(1u << sl.pin);
                    pinctrl_of(p, sl.pin) |= PORT_PULLUPEN_bm;
                    pulled_ = static_cast<uint8_t>(pulled_ | (1u << sl.pin));
                }
            }
            return;
        }
        if (mo.bonded) port.DIRCLR = static_cast<uint8_t>(1u << mo.pin);
        if (sc.bonded) port.DIRCLR = static_cast<uint8_t>(1u << sc.pin);
        if (sl.bonded) port.DIRCLR = static_cast<uint8_t>(1u << sl.pin);
        if (mi.bonded && c.drive_miso) {
            port.DIRSET = static_cast<uint8_t>(1u << mi.pin);
            driven_ = static_cast<uint8_t>(driven_ | (1u << mi.pin));
        } else if (mi.bonded) {
            port.DIRCLR = static_cast<uint8_t>(1u << mi.pin);
        }
    }

    /// Everything this driver drove goes back to being an input, and
    /// every pull-up it turned on goes back off.
    static void release_pins() {
        if (driven_ == 0 && pulled_ == 0) return;
        const char p = spi_port_letter(n, route_);
        if (p == '?' || !port_exists(p)) {
            driven_ = 0;
            pulled_ = 0;
            return;
        }
        volatile PORT_t& port = port_by_letter(p);
        port.DIRCLR = driven_;
        for (uint8_t i = 0; i < 8; ++i) {
            if (pulled_ & (1u << i)) {
                pinctrl_of(p, i) &= static_cast<uint8_t>(~PORT_PULLUPEN_bm);
            }
        }
        driven_ = 0;
        pulled_ = 0;
    }

    static inline SpiRoute route_ = SpiRoute::none;
    static inline SpiMode mode_ = SpiMode::mode0;
    static inline SpiRole role_ = SpiRole::host;
    static inline bool buffered_ = false;
    static inline uint8_t driven_ = 0;    ///< pins this driver set as outputs
    static inline uint8_t pulled_ = 0;    ///< pins whose pull-up this driver turned on
};

// ---- tasks ------------------------------------------------------------------

/*
 * SpiHost<n, route>
 *
 * The transfer ENGINE: the target-side half of the SPI stack, driven by
 * util/spi_bus.hpp (which owns arbitration and replies). This task owns
 * the wire: chip select, the D/C line of display-style devices, and the
 * byte pump under the SPI interrupt (no DMA on AVR Dx).
 *
 * Transaction descriptor (Request) - two phases in ONE chip-select
 * window, covering every device on the bench:
 *
 *   phase 1 (optional): cmd[cmd_len] transmitted with DC LOW
 *   phase 2 (optional): len bytes with DC HIGH, FULL-DUPLEX -
 *                       transmit tx[] (or 0xFF dummies if tx == null),
 *                       capture into rx[] (or discard if rx == null)
 *
 *   - display (ILI9341 & co.): cmd + tx, DC toggling inside the CS
 *     window, rx null
 *   - rx-only ADC (MCP3550):    no cmd, tx null, rx set
 *   - loopback / generic xfer:  tx and rx both set (true full duplex)
 *   - SD card:                  sequences of plain xfers
 *
 * CS is ACTIVE LOW and asserted/released by the engine around the whole
 * transaction; dc may be a null PinRef for DC-less devices. Buffer
 * ownership travels with the request (the client hands the spans off
 * until its SpiDone comes back). A zero-total-length request completes
 * on the spot without touching the wire (SpiDone still arrives).
 *
 * THE ENGINE RUNS THE INSTANCE IN BUFFER MODE (28.3.2.1.2): the
 * transmitter is a shift register plus one buffer, so two bytes are in
 * flight and the wire never waits for the software while the software
 * keeps up; the receiver is a two-entry FIFO in front of the shifter,
 * read at least every second transfer. Both completion styles are
 * written from that chapter's three rules and from the order 28.3.2.2.2
 * gives the flags of one transfer's end - DREIF as the buffered byte
 * moves into the shifter, then the received byte in the FIFO with RXCIF,
 * and TXCIF when nothing is left to send (measured: a DATA read made
 * one LDS after either DREIF or TXCIF is seen finds the byte there, at
 * every phase of the arrival):
 *
 *  - a write to DATA is legal while DREIF is set (rule 1): the first
 *    write of an idle transmitter goes straight to the shifter and the
 *    next to the buffer, so a burst opens with two writes and then waits
 *    DREIF before each one that follows;
 *  - a received byte lands in the FIFO when its transfer completes
 *    (rule 2), and the FIFO must be read at least every second transfer
 *    (rule 3). The engine READS BY COUNT: it keeps how many bytes it has
 *    written and not yet read and reads what the TRANSMITTER's flags
 *    prove has arrived - every written byte but the one shifting once
 *    DREIF stands, every written byte once TXCIF does -, never by RXCIF,
 *    which a read that empties the FIFO as a byte enters it loses (this
 *    file's header). The polled loop never makes that read: it leaves
 *    the newest byte in the FIFO until nothing shifts, so three bytes
 *    are outstanding there - one waiting, two in flight - and a late
 *    loop finds the FIFO full and the third byte held in the shifter,
 *    which the next read lets in before the next write starts a transfer
 *    (measured): BUFOVF cannot rise. The pump keeps two outstanding and
 *    reads right behind each arrival, so a late handler can make that
 *    read; it then drops the byte it is handed twice (isr()). A late
 *    loop or a late handler (an interrupt of the console's or the
 *    tick's inside a burst) idles the wire and loses nothing, at every
 *    division;
 *  - TXCIF says shifter and buffer are both empty: a CONDITION the
 *    register keeps (docs/avrdx/spi.md "Buffer mode"), so it speaks of a
 *    write only when it is cleared BEHIND that write, while the byte
 *    shifts and no earlier emptiness can raise it again. A polled burst
 *    clears it behind its last write with the two stores masked together
 *    - an interrupt between them could let the byte finish and the clear
 *    wipe the one edge the tail waits for -; the pump's launch clears it
 *    behind its two writes, masked with them, and its handler behind the
 *    first write of every refill.
 *
 * Between transactions the transmitter is idle and the receive FIFO
 * empty: every burst and every pumped phase end so, and init() and
 * recover() make it so (one DATA read whatever the flag says, then the
 * FIFO drained).
 *
 * The POLLED path completes inside start() and reads every field of the
 * request through the reference it was lent (the arbiter's contract:
 * the slot stays as it is until the dispatch ends) - it copies nothing.
 * The PUMP (polled false) copies at launch what its tenure needs - the
 * two pins, the data phase's spans and length - into the engine's own
 * state, and runs on RXCIE with TXCIE beside it: the handler reads by
 * the same count and writes the next bytes until two are in flight. A
 * read the silicon did not take costs one missed edge: the two in flight
 * complete, TXCIF rises with RXCIF down, and the handler drops the
 * repeated byte and reads both - a hiccup, not a stall.
 *
 * ISR wiring (app glue, as usual):
 *   ISR(SPI0_INT_vect) {
 *       if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{brio::spi_ok}); }
 *   }
 */
template <uint8_t n, SpiRoute route = SpiRoute::def>
class SpiHost {
    using S = Spi<n>;
    static_assert(!spi_route_on_updi_pin(n, route),
                  "SPI0 ALT6 puts SS on PF7, the UPDI pin: a GPIO only with "
                  "FUSE.SYSCFG0.UPDIPINCFG = 0, which leaves the part unreachable "
                  "without a high-voltage programmer - refused");
    static_assert(!spi_route_crosses_ports(n, route),
                  "this route's signals sit on two ports (AVR EA SPI0 ALT3: PA0, PA1, "
                  "PC0, PC1), which the pin hand-over does not drive yet");
    static_assert(spi_route_exists(n, route),
                  "this part has not got this SPI route, or this package does not bond it "
                      "(AVR DA/DB: 28/32 pins have DEFAULT only, SPI1 ALT2 is refused on 48-pin "
                      "parts by DB errata 2.11.1; AVR EA: code 2 is reserved, ALT1 needs 48 pins)");

public:
    SpiHost() = delete;

    using Resource = S;
    static constexpr SpiRoute pin_route = route;

    /// Does this package bond everything a host on this route needs?
    static constexpr bool available =
        spi_route_exists(n, route) &&
        (route == SpiRoute::none || spi_pin(n, route, SpiSignal::sck).bonded);

    /// THE LAYOUT: 21 bytes and no padding on this core (every type here
    /// is byte-aligned): two PinRefs of three, three pointers of two, a
    /// byte and a word of lengths, the reply's two, four bytes of
    /// settings. The first fifteen are the TENURE's - what the pump
    /// still needs once start() has returned - and lie together; the
    /// rest is consumed inside start(). The field names are the
    /// contract every stratum shares (docs/design/spi-bus.md).
    struct Request {
        PinRef cs;             ///< asserted low around the transaction
        PinRef dc;             ///< display D/C line; null = no such pin
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        uint8_t cmd_len;
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len;          ///< phase 2 length
        ReplyTo<SpiDone> reply;
        // Per-transaction bus configuration: on a SHARED bus every
        // device names its own speed and mode in the request (ILI9481
        // at 6 MHz, XPT2046 capped at ~2.5 MHz, ...); the engine folds
        // both into the two register bytes the block takes, compares
        // them with the pair it last wrote, and reprograms only on a
        // change - nothing per byte. The rate is a DIVISION of CLK_PER,
        // so it follows a clock change by itself; max_sck_hz() is the
        // ceiling that clamps it.
        SpiClock clock = SpiClock::div16;
        SpiMode mode = SpiMode::mode0;
        // Completion style, also the client's call: false = the
        // interrupt pump (the kernel keeps running between bytes -
        // right for slow clocks and short transfers); true = POLLED
        // inside start(), completing synchronously. The polled loop is
        // wire-bound from CLK_PER/4 (a byte is 32 CPU cycles there and
        // the loop fewer, counted in docs/avrdx/spi.md); the pump keeps
        // two bytes in flight, so the wire runs while its handler does,
        // but one interrupt per byte costs more than a byte at the fast
        // divisions and the pump is CPU-bound there. The price of
        // polling is that THIS dispatch blocks for the whole transfer
        // (bounded, chosen here); global interrupts stay enabled
        // throughout - the SPI's own interrupt is never armed for a
        // polled request.
        bool polled = false;
        // Chip-select setup: microseconds the engine waits between
        // asserting CS and the first SCK edge. Most devices need tens of
        // ns (what the code path takes anyway). A device waking
        // from shutdown on CS may need more: the MCP3550 datasheet
        // specifies tRDY <= 50 ns but only says "an internal power-up
        // delay must be observed" when exiting Shutdown (DS20001950F
        // 5.2) - MEASURED: SDO drives ~4 us after CS falls, and a frame
        // clocked 1.5 us after CS is lost (0x7FFFFF), 3.5 us is enough,
        // dac_adc uses 10. Spent spinning in start(), main context,
        // bounded by this byte; tested before the delay is called, so
        // a zero costs one branch.
        uint8_t cs_setup_us = 0;
    };
    static_assert(std::is_trivially_copyable_v<Request>);
    static_assert(sizeof(Request) == 21, "the request is 21 bytes with no padding on this core");

    /**
     * Host, MSB first, SSD set (the SS position stays free for a GPIO -
     * chip selects are the engine's, not the peripheral's), BUFFER MODE
     * on. Call after clock init, before sei(). CS/DC pins are configured
     * by their owners (the device clients), not here; clock and mode
     * travel per-request. `clock` is the app's brio::Clock tag: the
     * per-request cs_setup_us delay is timed from Clock::hz.
     *
     * `max_sck_hz` is an optional CEILING for the whole bus: with it set
     * the engine slows any request that would exceed it (and re-picks
     * the division after a clock change), which is what makes rebase()
     * meaningful. 0 = no ceiling, the request's division is used as it
     * stands.
     */
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its cs_setup timing and its SCK ceiling "
                      "would go stale on a clock change");
        if constexpr (!available) {
            (void)clock;
            (void)max_sck_hz;
            return false;
        } else {
            ceiling_hz_ = max_sck_hz;
            rebase(clock_hz(clock));
            if (max_sck_hz != 0 && !ceiling_) return false;   // even /128 is too fast
            if (!S::init({.route = route, .role = SpiRole::host, .mode = SpiMode::mode0,
                          .clock = ceiling_ ? *ceiling_ : SpiClock::div16,
                          .client_select_disable = true, .buffer_mode = true})) {
                return false;
            }
            // The kept pair is what the registers HOLD, read back once:
            // apply() can then never disagree with init() about the
            // bits it does not fold (DORD, BUFWR) or the ones it does.
            ctrla_ = S::regs().CTRLA;
            ctrlb_ = S::regs().CTRLB;
            // Neither the disable in the resource's init nor its store
            // of one to RXCIF empties the receive FIFO (the store drops
            // the flag and leaves the bytes): what a former user left
            // there is read out here.
            flush();
            ahead_ = 0;
            to_write_ = 0;
            in_cmd_ = false;
            return true;
        }
    }

    /// The peripheral clock changed (DynamicClock fan-out): the SCK
    /// divisions travel per request and scale with CLK_PER by
    /// themselves, so what is recomputed here is the cs_setup_us timing
    /// base and - when a ceiling was asked for - the slowest division
    /// that still honours it. The bus must be IDLE: a transfer in
    /// flight keeps the old division for its remaining bytes.
    static void rebase(uint32_t hz) {
        clk_per_hz_ = hz;
        cycles_per_us_ = cycles_per_us(hz);
        ceiling_ = ceiling_hz_ ? spi_clock_for(hz, ceiling_hz_) : std::optional<SpiClock>{};
    }

    /// The SCK ceiling in force (0 = none) and the division it resolves
    /// to at the peripheral clock last seen.
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    /// What SCK a request at this division really runs at.
    static uint32_t sck_hz(SpiClock c) { return spi_sck_hz(clk_per_hz_, clamp(c)); }
    /// The fastest division at or below `max_sck_hz` at the peripheral
    /// clock last seen - `spi_clock_for()` with this engine's own rate
    /// filled in; nothing when even the slowest is too fast.
    static std::optional<SpiClock> clock_for(uint32_t max_sck_hz) {
        return spi_clock_for(clk_per_hz_, max_sck_hz);
    }

    /// The engine's completion status, read by the app glue for the
    /// TransferDone payload: always spi_ok on this engine, which has no
    /// DMA path and therefore no fault of its own to report - the verb
    /// exists so the glue is spelled the same way on every stratum.
    static constexpr uint8_t status() { return spi_ok; }

    /// FOR CALLERS THAT FRAME THE SELECT WINDOW THEMSELVES (Request.cs
    /// null). start() applies a request's mode before it asserts the
    /// request's own cs, so an engine-owned select window always opens
    /// with SCK settled at the new idle level - but a caller driving CS
    /// by hand inverts that order, and a mode change is a CPOL FLIP ON
    /// THE WIRE (the SCK pad moves to the new idle level while the
    /// peripheral is off): flipped inside an open select window it is
    /// one extra edge a selected client can count into the character.
    /// Prime FIRST, then assert the select.
    static void prime(SpiMode mode, SpiClock clock) { apply(mode, clamp(clock)); }

    /// Begin a transaction (called by SpiBus from main context).
    /// Returns true when the transaction completed synchronously
    /// (polled requests, and the degenerate zero-length one); false
    /// when it runs on the ISR and a TransferDone will follow.
    ///
    /// The request is LENT for this call and read through `r`: the
    /// polled path, which completes before returning, keeps nothing;
    /// the pump copies its tenure's fields in launch().
    static bool start(const Request& r) {
        if (static_cast<uint16_t>(r.cmd_len) + r.len == 0) {
            return true;  // nothing to move: complete on the spot
        }
        apply(r.mode, clamp(r.clock));
        if (r.cmd_len != 0) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();                      // assert, active low
        if (r.cs_setup_us != 0) {
            delay_us_runtime(cycles_per_us_, r.cs_setup_us);
        }
        if (!r.polled) {
            launch(r);
            return false;
        }
        // Polled: the SPI's own interrupt is not armed (the pump arms it
        // at launch and its handler disarms it at the end, so an idle
        // bus has it off), global interrupts STAY ENABLED - the
        // console, the tick, anything else preempts these loops freely,
        // and the drain-before-write discipline of burst() is what
        // makes that safe. The loans are VIEWS: .get() hands out the
        // raw pointer the loops index (Borrowed is not a container).
        if (r.cmd_len != 0) {
            burst<true, false>(r.cmd.get(), nullptr, r.cmd_len);
            r.dc.set();                    // the data phase, even an empty one
        }
        if (r.len != 0) {
            // Shape-specialized loops: the per-byte budget at CLK_PER/4
            // is 32 cycles, so the tx/rx null tests are decided once
            // here and never inside a loop.
            const uint8_t* const tx = r.tx.get();
            uint8_t* const rx = r.rx.get();
            if (rx == nullptr) {
                if (tx != nullptr) {
                    burst<true, false>(tx, nullptr, r.len);      // bulk write
                } else {
                    burst<false, false>(nullptr, nullptr, r.len); // clocks only
                }
            } else if (tx != nullptr) {
                burst<true, true>(tx, rx, r.len);                 // full duplex
            } else {
                burst<false, true>(nullptr, rx, r.len);           // bulk read
            }
        }
        r.cs.set();                        // release: transaction done
        return true;
    }

    /**
     * SPI interrupt body - call from ISR(SPIn_INT_vect). The pump runs
     * on RXCIE and TXCIE. The body reads the flags once, reads out of the
     * FIFO every byte the transmitter's flags prove has arrived (all it
     * wrote once TXCIF stands, all but the one shifting once DREIF does,
     * all but two otherwise) and stores them where the request wanted
     * them, then writes the next bytes until two are in flight again,
     * clearing TXCIF behind the first. In the steady state that is one
     * byte read and one written per RXCIF; a handler that ran late finds
     * the transmitter idle and TXCIF up, reads both and writes two. A
     * read of this handler that emptied the FIFO as the next byte entered
     * it was not taken (this file's header): RXCIF stays down, the next
     * edge is TXCIF's once the two in flight are done, and an entry with
     * TXCIF up and RXCIF down while bytes are due is that case and no
     * other - the flags are read some twenty cycles past the edge that
     * took the vector, so they have settled - and reads the repeated byte
     * once more and drops it. Returns true when the transaction just
     * completed (CS released): the edge on which the glue posts
     * TransferDone to the bus AO.
     */
    [[gnu::always_inline]] static bool isr() {
        auto& s = S::regs();
        const uint8_t f = s.INTFLAGS;
        uint8_t keep = 0;                  // written bytes not yet received
        uint8_t a = ahead_;
        if ((f & SPI_TXCIF_bm) == 0) {
            keep = (f & SPI_DREIF_bm) != 0 ? 1 : 2;
        } else if ((f & SPI_RXCIF_bm) == 0 && a != 0) {
            // Everything written has arrived and RXCIF says nothing has:
            // a read of this handler met an arrival, and the byte it
            // returned is still at the FIFO's head. Read again and drop.
            (void)s.DATA;
        }
        while (a > keep) {                 // read by count, whatever RXCIF says
            const uint8_t in = s.DATA;
            if (in_ != nullptr) {
                *in_++ = in;
            }
            --a;
        }
        if (to_write_ == 0) {
            if (a != 0) {                  // the phase's last bytes still shift
                ahead_ = a;
                return false;
            }
            if (in_cmd_) {                 // the command phase is over
                in_cmd_ = false;
                dc_.set();
                out_ = tx_;
                in_ = rx_;
                to_write_ = len_;
            }
            if (to_write_ == 0) {
                cs_.set();                 // release: transaction done
                s.INTCTRL = 0;             // the pump's enables, off until the next launch
                ahead_ = 0;
                return true;
            }
        }
        if (a < 2) {                       // two in flight again
            write_next();
            s.INTFLAGS = SPI_TXCIF_bm;     // behind the write: TXCIF speaks of it
            ++a;
            if (a < 2 && to_write_ != 0) {
                write_next();
                ++a;
            }
        }
        ahead_ = a;
        return false;
    }

    /// The work-around for a wedged host: a lost completion, or a
    /// demotion mid-transfer (SS driven low with SSD clear stops the pump
    /// dead, 28.3.2.1.3 - measured; init() sets SSD, so here it is the
    /// chapter's note honoured and not a path a request can reach). The
    /// pump's interrupt is silenced, the Host role re-armed if the
    /// hardware dropped it, the bytes still in flight waited for (bounded:
    /// two transfers at CLK_PER/128 with room to spare - a clock that
    /// does not run ends the wait), the receive FIFO read out whatever
    /// RXCIF says, the select window closed; start() reprograms mode and
    /// clock per request, so nothing else needs saving. The verb a timed
    /// SpiBus calls (util/bus_master.hpp).
    static void recover() {
        if constexpr (available) {
            auto& s = S::regs();
            s.INTCTRL = 0;
            if (S::demoted()) {
                S::restore_host();
            }
            if (ahead_ != 0) {
                // A turn is at least six cycles; two transfers at the
                // slowest division are 2 x (8 x 128 + 2).
                for (uint16_t i = 0; i < 512u && (s.INTFLAGS & SPI_TXCIF_bm) == 0; ++i) {
                }
            }
            flush();
            cs_.set();
            in_cmd_ = false;
            ahead_ = 0;
            to_write_ = 0;
        }
    }

    /// Hand the route's pins back (the resource's teardown).
    static void release() { S::release(); }

private:
    /// The ceiling is a slowest division: the enum is ordered by
    /// division, so a request faster than it is a smaller member.
    [[gnu::always_inline]] static SpiClock clamp(SpiClock c) {
        if (ceiling_ && c < *ceiling_) return *ceiling_;
        return c;
    }

    /// The two register bytes a request's settings fold into: the host's
    /// fixed bits and the request's rate in CTRLA, the engine's fixed
    /// bits (buffer mode, SSD) and the request's mode in CTRLB.
    static constexpr uint8_t ctrla_of(SpiClock c) {
        return static_cast<uint8_t>(SPI_MASTER_bm | SPI_ENABLE_bm | spi_presc_bits(c));
    }
    static constexpr uint8_t ctrlb_of(SpiMode m) {
        return static_cast<uint8_t>(SPI_BUFEN_bm | SPI_SSD_bm | static_cast<uint8_t>(m));
    }

    /// apply(): the fold, one compare per byte, and nothing written on
    /// the unchanged path - the common one on a bus of one device.
    [[gnu::always_inline]] static void apply(SpiMode mode, SpiClock clock) {
        const uint8_t a = ctrla_of(clock);
        const uint8_t b = ctrlb_of(mode);
        if (a != ctrla_ || b != ctrlb_) {
            reprogram(a, b);
        }
    }

    /// The change path, out of line. SCK must already sit at the new
    /// mode's idle level (CPOL) when CS falls: devices that latch their
    /// SPI mode from SCK at the CS edge (MCP3550: mode 0,0 vs 1,1)
    /// otherwise start the transaction in the wrong mode. The AVR SPI
    /// updates the SCK output level when it is ENABLED and at every
    /// transfer - NOT on a CTRLB write while it is enabled (seen on the
    /// analyzer: SCK still low 10 us after CS fell on the first mode-3
    /// request after init; a known AVR quirk). So a CPOL change is
    /// applied with the peripheral disabled: preset the SCK pin's
    /// PORT.OUT to the new idle level (what the pin shows while the SPI
    /// is off - no glitch), disable, write the mode, re-enable. Three
    /// register writes, no clock edges on the bus, only when the
    /// polarity changes between transactions; a rate or a CPHA change
    /// is two stores with the instance enabled, as the resource's own
    /// clock() and mode() verbs do.
    [[gnu::noinline]] static void reprogram(uint8_t a, uint8_t b) {
        auto& s = S::regs();
        if (((b ^ ctrlb_) & 0x02u) != 0) {   // MODE bit 1 is CPOL
            if constexpr (route != SpiRoute::none) {
                constexpr SpiPin sc = spi_pin(n, route, SpiSignal::sck);
                if constexpr (sc.bonded) {
                    volatile PORT_t& port = port_by_letter(sc.port);
                    if ((b & 0x02u) != 0) port.OUTSET = static_cast<uint8_t>(1u << sc.pin);
                    else port.OUTCLR = static_cast<uint8_t>(1u << sc.pin);
                }
            }
            s.CTRLA = static_cast<uint8_t>(a & ~SPI_ENABLE_bm);
        }
        s.CTRLB = b;
        s.CTRLA = a;
        ctrla_ = a;
        ctrlb_ = b;
    }

    /// One polled burst of count >= 1 bytes, the loop of 28.3.2.1.2 read
    /// by count, with the FIFO NEVER EMPTIED WHILE A BYTE SHIFTS: the
    /// first byte into the idle shifter and the second into the buffer;
    /// then, per byte, DREIF waited for - the byte before has moved into
    /// the shifter, so every byte before THAT one is in the FIFO -, the
    /// older of the two newest read whatever RXCIF says, so that one
    /// stays, and the next written: two in flight and one waiting, the
    /// wire busy while this loop runs its turn. A read that empties the
    /// FIFO as a byte enters it is the one the silicon does not take
    /// (this file's header); a read that leaves a byte behind never meets
    /// that cycle (docs/avrdx/spi.md). The last write is masked together
    /// with TXCIF's clear behind it, so that the tail's TXCIF means "every
    /// byte written is in the FIFO, or held in the shifter behind a full
    /// one", and the last one, two or three are read with nothing left
    /// to arrive. The shape is a template parameter so that no null test
    /// runs inside: `has_out` false clocks 0xFF, `has_in` false discards.
    /// The spins are not bounded: with SSD set nothing can take the clock
    /// generator from this host (28.3.2.1.3), and the host clocks itself,
    /// so a transfer written always completes.
    template <bool has_out, bool has_in>
    [[gnu::always_inline]] static void burst(const uint8_t* out, uint8_t* in, uint16_t count) {
        (void)out;
        (void)in;
        auto& s = S::regs();
        // The last byte is taken before the loop, so that the source
        // pointer ends with it.
        uint8_t last = 0xFF;
        if constexpr (has_out) {
            last = out[count - 1u];
        }
        uint8_t tail = 1;                  // bytes not yet read at the last write
        if (count != 1) {
            tail = 2;
            s.DATA = next_out<has_out>(out);       // the idle shifter: DREIF stays up
            if (count != 2) {
                tail = 3;
                s.DATA = next_out<has_out>(out);   // the buffer
                wait_dre();                        // the first in the FIFO: it stays
                if (count != 3) {
                    s.DATA = next_out<has_out>(out);
                    count = static_cast<uint16_t>(count - 3u);
                    for (;;) {
                        wait_dre();
                        read_in<has_in>(in);       // the older of two: one stays
                        // An empty asm the count passes through, so that
                        // gcc keeps it as the loop's counter: otherwise it
                        // rewrites the exit as a compare against a
                        // pointer's end or against the count it came in
                        // with, and shuffles the pointers through other
                        // registers to do it.
                        __asm__("" : "+r"(count));
                        if (--count == 0) {
                            break;
                        }
                        s.DATA = next_out<has_out>(out);
                    }
                }
            }
        }
        {
            // The last byte, then TXCIF cleared behind it while it shifts.
            // Masked: with a handler between the two stores the byte could
            // finish and the clear wipe the only edge the tail waits for.
            AvrPlatform::CriticalSection cs;
            s.DATA = last;
            s.INTFLAGS = SPI_TXCIF_bm;
        }
        while ((s.INTFLAGS & SPI_TXCIF_bm) == 0) {
        }
        do {
            read_in<has_in>(in);
        } while (--tail != 0);
    }

    /// DREIF: room in the transmit buffer.
    [[gnu::always_inline]] static void wait_dre() {
        while ((S::regs().INTFLAGS & SPI_DREIF_bm) == 0) {
        }
    }

    /// A burst's next byte: the source's, or the dummy a shape without
    /// one clocks.
    template <bool has_out>
    [[gnu::always_inline]] static uint8_t next_out(const uint8_t*& out) {
        if constexpr (has_out) {
            return *out++;
        } else {
            return 0xFF;
        }
    }

    /// One byte out of the receive FIFO, the count having proved it is
    /// there: stored, or discarded by a shape that wants none.
    template <bool has_in>
    [[gnu::always_inline]] static void read_in(uint8_t*& in) {
        const uint8_t v = S::regs().DATA;
        if constexpr (has_in) {
            *in++ = v;
        } else {
            (void)v;
        }
    }

    /// The receive FIFO emptied with the transmitter idle (the
    /// resource's flush_receive()).
    static void flush() { S::flush_receive(); }

    /// The pump's launch: the tenure's fields copied out of the lent
    /// request (the pins; the data phase's spans and length, kept aside
    /// while a command phase runs), up to two bytes written - the
    /// shifter's and the buffer's - with the write-one flags cleared
    /// behind them, the count set, and RXCIE and TXCIE armed last, over a
    /// state already whole. The two stores and the clear are masked
    /// together: a handler between the two writes could let the first
    /// byte finish and raise TXCIF while the second had not yet gone, and
    /// a TXCIF that predates a write tells the handler that byte has
    /// arrived (measured: the read made for it found nothing, and the
    /// byte, arriving after, was left over). A command phase never writes
    /// ahead of its own end: the D/C edge must land between its last byte
    /// and the data phase's first, so the data phase starts in the
    /// handler.
    static void launch(const Request& r) {
        cs_ = r.cs;
        dc_ = r.dc;
        if (r.cmd_len != 0) {
            in_cmd_ = true;
            out_ = r.cmd.get();
            in_ = nullptr;
            to_write_ = r.cmd_len;
            tx_ = r.tx.get();
            rx_ = r.rx.get();
            len_ = r.len;
        } else {
            in_cmd_ = false;
            out_ = r.tx.get();
            in_ = r.rx.get();
            to_write_ = r.len;
        }
        auto& s = S::regs();
        const uint8_t first = take_next();
        uint8_t a = 1;
        uint8_t second = 0;
        if (to_write_ != 0) {
            second = take_next();
            a = 2;
        }
        {
            AvrPlatform::CriticalSection cs;
            s.DATA = first;                // the idle shifter
            if (a == 2) {
                s.DATA = second;           // the buffer
            }
            s.INTFLAGS = static_cast<uint8_t>(SPI_TXCIF_bm | SPI_SSIF_bm | SPI_BUFOVF_bm);
        }
        ahead_ = a;
        s.INTCTRL = static_cast<uint8_t>(SPI_RXCIE_bm | SPI_TXCIE_bm);
    }

    /// The current phase's next byte, counted out: the source's, or the
    /// dummy where the phase has none.
    [[gnu::always_inline]] static uint8_t take_next() {
        uint8_t v = 0xFF;
        if (out_ != nullptr) {
            v = *out_++;
        }
        --to_write_;
        return v;
    }

    /// The next byte of the current phase into DATA: the source's, or a
    /// dummy where the phase has none.
    [[gnu::always_inline]] static void write_next() { S::regs().DATA = take_next(); }

    // The tenure: what the pump keeps of a request past start().
    static inline PinRef cs_{};
    static inline PinRef dc_{};
    static inline const uint8_t* tx_ = nullptr;   ///< the data phase, waiting behind a command phase
    static inline uint8_t* rx_ = nullptr;
    static inline uint16_t len_ = 0;
    // The current phase.
    static inline const uint8_t* out_ = nullptr;  ///< next byte to write; null = 0xFF
    static inline uint8_t* in_ = nullptr;         ///< next received byte's home; null = discard
    static inline uint16_t to_write_ = 0;
    /// Bytes written and not yet read out of the FIFO: 0, 1 or 2.
    static inline uint8_t ahead_ = 0;
    static inline bool in_cmd_ = false;
    // The register pair last written (init() reads it back once).
    static inline uint8_t ctrla_ = 0;
    static inline uint8_t ctrlb_ = 0;
    static inline uint8_t cycles_per_us_ = 1;   // from Clock::hz at init()
    static inline uint32_t clk_per_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<SpiClock> ceiling_{};
};

/*
 * SpiClient<n, route>
 *
 * The other end of the wire (28.3.2.2): SS, MOSI and SCK are inputs, the
 * host sets the pace, and the only thing this side controls is WHAT it
 * has ready to shift out when the next clock arrives.
 *
 *  - NORMAL mode: one byte in flight. preload() while SS is high, then
 *    every byte the host clocks in arrives with IF set and the byte
 *    written last goes out. A write during a transfer is IGNORED and
 *    sets WRCOL (a client cannot corrupt a frame the way a host can).
 *  - BUFFER mode: two transmit levels and a two-deep receive FIFO, with
 *    DREIF/TXCIF/RXCIF/BUFOVF instead of IF. `buffer_wait` (BUFWR)
 *    decides whether the FIRST byte the host clocks out is the one the
 *    application wrote (BUFWR = 1: the write goes straight to the
 *    shifter while SS is high) or a DUMMY equal to whatever the shift
 *    register held (BUFWR = 0). Errata clarification 3.5.2 warns that a
 *    client in buffer mode near the maximum SCK may not set up data in
 *    time for the first sample edge of a back-to-back transfer.
 *
 * The SS pin is the frame boundary: driven high, the state machine is
 * RESET and a partial byte is lost (28.3.2.2.3). selected() reads it.
 *
 * Its unit is the RUN - answer(), exchange(), receive() serve a
 * transaction's bytes in one polled call - and the per-byte verbs are the
 * degenerate case. In buffer mode the runs READ BY COUNT and never by
 * RXCIF: the client's transmitter flags count the host's transfers
 * (28.3.2.2.2: DREIF rises after each transfer that moves the transmit
 * buffer into the shifter, RXCIF one cycle after it, TXCIF one cycle
 * after RXCIF once shifter and buffer are both empty), so a run that
 * refills the buffer on every DREIF knows how many bytes have arrived,
 * and reads the older of the two newest - the FIFO is never emptied while
 * a byte may enter it, the read the silicon does not take (this file's
 * header) is never made, and the read follows the flag by the test of
 * the late flags, past RXCIF's one cycle. The cure by RXCIF - a read,
 * the flag tested, one more read when it is down - is no cure: an
 * arrival between the test and the second read is taken by it and
 * dropped (test_avr_spi letter t, docs/avrdx/spi.md).
 */
template <uint8_t n, SpiRoute route = SpiRoute::def>
class SpiClient {
    using S = Spi<n>;
    static_assert(!spi_route_on_updi_pin(n, route),
                  "SPI0 ALT6 puts SS on PF7, the UPDI pin: a GPIO only with "
                  "FUSE.SYSCFG0.UPDIPINCFG = 0, which leaves the part unreachable "
                  "without a high-voltage programmer - refused");
    static_assert(!spi_route_crosses_ports(n, route),
                  "this route's signals sit on two ports (AVR EA SPI0 ALT3: PA0, PA1, "
                  "PC0, PC1), which the pin hand-over does not drive yet");
    static_assert(spi_route_exists(n, route),
                  "this part has not got this SPI route, or this package does not bond it "
                      "(AVR DA/DB: 28/32 pins have DEFAULT only, SPI1 ALT2 is refused on 48-pin "
                      "parts by DB errata 2.11.1; AVR EA: code 2 is reserved, ALT1 needs 48 pins)");

public:
    SpiClient() = delete;

    using Resource = S;
    static constexpr SpiRoute pin_route = route;

    /// A client needs real SCK, MOSI and SS pins: there is nothing for a
    /// pinless one to be clocked by.
    static constexpr bool available =
        spi_route_exists(n, route) && route != SpiRoute::none &&
        spi_pin(n, route, SpiSignal::sck).bonded &&
        spi_pin(n, route, SpiSignal::mosi).bonded &&
        spi_pin(n, route, SpiSignal::ss).bonded;

    struct Options {
        SpiMode mode = SpiMode::mode0;
        bool lsb_first = false;
        bool buffer_mode = false;
        bool buffer_wait = false;   ///< BUFWR: the first write skips the dummy byte
        bool drive_miso = true;     ///< the answer line; the pad tri-states while SS is high
    };

    /// Configure the instance as a client. False (nothing programmed)
    /// when this package cannot bond the route's pins. Called with the
    /// client deselected: in buffer mode the receive FIFO is emptied
    /// (the resource's flush_receive()), because the resource's init()
    /// leaves what a former user left there under a lowered RXCIF and a
    /// client that waits for RXCIF would never hear another byte.
    static bool init(Options o = {}) {
        if constexpr (!available) {
            (void)o;
            return false;
        } else {
            if (!S::init({.route = route, .role = SpiRole::client, .mode = o.mode,
                          .lsb_first = o.lsb_first,
                          .client_select_disable = true,   // SSD is a host bit; keep it out of the way
                          .buffer_mode = o.buffer_mode,
                          .buffer_wait = o.buffer_wait,
                          .drive_miso = o.drive_miso})) {
                return false;
            }
            if (o.buffer_mode) S::flush_receive();
            return true;
        }
    }

    /// The SS pin's level: LOW means this client is selected and the
    /// next SCK edge shifts. Reads the pin through PORT (the SPI does
    /// not report it outside buffer mode's SSIF).
    static bool selected() {
        if constexpr (!available) {
            return false;
        } else {
            constexpr SpiPin sl = spi_pin(n, route, SpiSignal::ss);
            return (port_by_letter(sl.port).IN & (1u << sl.pin)) == 0;
        }
    }

    /// HOW MANY ANSWERS MUST BE QUEUED AHEAD of the frame the host is
    /// about to clock for them to reach the wire in order - the one
    /// integer that differs between this family's client pump and the
    /// other strata's. ONE here: the byte written last goes out with the
    /// next clock, so a write made in the gap after IF is in time (with
    /// BUFWR two can be queued, but one is what the pump needs).
    static constexpr uint8_t frames_ahead = 1;

    /// Load what the next clocked byte will carry out. While SS is high
    /// this is the answer prepared for the coming frame; in normal mode
    /// during a transfer it is IGNORED and sets WRCOL.
    static void preload(uint8_t v) { S::write(v); }

    /// A byte if one has arrived (IF, or RXCIF in buffer mode).
    static std::optional<uint8_t> poll() { return S::poll(); }
    /// Wait (bounded) for a byte the host clocks in.
    static std::optional<uint8_t> wait(uint32_t spins = 500'000u) { return S::wait(spins); }

    /// Prepare `out` for the next frame and return the byte the host
    /// clocks in - the client's half of one exchange.
    static std::optional<uint8_t> exchange(uint8_t out, uint32_t spins = 500'000u) {
        S::write(out);
        return S::wait(spins);
    }

    /// Buffer mode: room for another answer / a frame finished / bytes
    /// lost because the two-deep FIFO was not drained.
    static bool ready_for_data() { return S::dre_flag(); }
    static bool transfer_complete() { return S::txc_flag(); }
    static bool overflow() { return S::overflow_flag(); }
    static bool write_collision() { return S::write_collision(); }
    /// INTFLAGS as it reads, in the layout the instance runs.
    static uint8_t flags() { return S::flags(); }

    // ---- the polled runs -----------------------------------------------------
    //
    // A client's unit is the RUN: the answers to a transaction, or what it
    // brings, served polled in one call. What bounds a run on this block
    // is one level of slack on each side (28.3.2.2.2): an answer written on
    // the DREIF that marks transfer t's end goes out in transfer t + 2, and
    // the FIFO, holding the newest byte and the one before it, has room for
    // the byte of transfer t + 1 only once the older is read. So a TURN -
    // the flag seen, the older byte read, the next answer written - has
    // the whole of transfer t + 1 to land in, eight SCK periods. A turn
    // later than that finds the transmitter run dry, and the transfer that
    // dried it raised no DREIF, so the count is no longer the host's.
    // TXCIF says so - cleared behind the preload, it can rise before the
    // run's last transfer only that way -, BUFOVF beside it, and the run
    // ends `late` on them rather than read by a count it no longer has.
    // The cost of each turn is counted in the release listing and measured
    // against the wire in docs/avrdx/spi.md.
    //
    // A run is called with the select HIGH and the transmitter empty (a
    // fresh init(), or a former run that ended complete): the preload goes
    // into the shifter (BUFWR) or the buffer before the host can clock.
    // The select pad is the window: a run waits for it to fall (or for the
    // first transfer's flag) before it takes a high pad for the close.

    /// How a run ended.
    enum class RunEnd : uint8_t {
        complete,   ///< every byte of the run moved (exchange, receive)
        closed,     ///< the host raised the select
        stalled,    ///< the spin budget ran out first (or the transmitter was not empty)
        late,       ///< a turn missed its transfer: the transmitter ran dry (TXCIF, buffer
                    ///< mode) or its write met the next transfer (WRCOL, normal mode)
    };

    /// What a run did: `count` bytes - answers loaded for answer(), bytes
    /// received in order for exchange() and receive() - and how it ended.
    struct Run {
        uint16_t count;
        RunEnd end;
    };

    /**
     * ANSWER a transaction with `out`, polled on DREIF alone, reading
     * nothing. In buffer mode the first answer is written with the select
     * still high - into the shifter under BUFWR, the buffer's second
     * beside it; without BUFWR into the buffer, and the shifter's leftover
     * leads (28.3.2.2.2's dummy) - and every later one on the DREIF that
     * marks a transfer's end. In normal mode the first is loaded with the
     * select high and every later one on IF, in the host's inter-byte gap
     * (28.3.2.2.1: a write during a transfer is ignored and raises WRCOL,
     * which ends the run `late`).
     *
     * The run ENDS WHEN THE HOST RAISES THE SELECT, however many bytes it
     * clocked: fewer than the run, and `count` says how many answers were
     * loaded; more, and the bytes past the run carry what the shifter is
     * left with. NOTHING IS READ while it runs, so the FIFO fills and what
     * the host sent is lost; the tail empties the FIFO (flush_receive) so
     * the next run starts clean.
     *
     * `spins` bounds the whole run: one count per four idle polls, charged 256 at a time.
     */
    template <typename T>
    static Run answer(std::span<const T> out, uint32_t spins) {
        static_assert(run_byte<T>, "the AVR DA/DB SPI moves eight-bit characters (28.3): a "
                                   "run carries uint8_t");
        static_assert(available, "a client run needs the route's SCK, MOSI and SS pins, "
                                 "bonded on this package");
        const uint16_t len = static_cast<uint16_t>(out.size());
        if (len == 0) return {0, RunEnd::complete};
        return S::buffer_mode() ? answer_buffered(out.data(), len, spins)
                                : answer_normal(out.data(), len, spins);
    }

    /**
     * EXCHANGE: answer with `out` and keep what the host sends in `in`,
     * the run's length the shorter of the two. The answers go as
     * answer()'s; in buffer mode each turn then reads the byte BEFORE the
     * one that just arrived (the count above), and the last two are read
     * once the last transfer is in - TXCIF under BUFWR, the last DREIF
     * without. In normal mode a turn reads the byte IF announced, after
     * the answer's write.
     *
     * Ends `complete` when the last byte is in; `closed` when the host
     * raised the select first, what arrived before it read into `in`
     * (`count` says how far); `stalled` on the budget; `late` as above,
     * the bytes received in order before it read out. The receive FIFO
     * is emptied at the start, with the select high, so `in` holds this
     * transaction alone.
     */
    template <typename T>
    static Run exchange(std::span<const T> out, std::span<T> in, uint32_t spins) {
        static_assert(run_byte<T>, "the AVR DA/DB SPI moves eight-bit characters (28.3): a "
                                   "run carries uint8_t");
        static_assert(available, "a client run needs the route's SCK, MOSI and SS pins, "
                                 "bonded on this package");
        const uint16_t len = static_cast<uint16_t>(out.size() < in.size() ? out.size()
                                                                         : in.size());
        if (len == 0) return {0, RunEnd::complete};
        return S::buffer_mode() ? serve_buffered<true>(out.data(), 0, in.data(), len, spins)
                                : serve_normal<true>(out.data(), in.data(), len, spins);
    }

    /**
     * RECEIVE a run into `in`. In buffer mode the run still WRITES, the
     * byte `fill` on every turn: the transmitter's flags are this block's
     * only count of the host's transfers, and a buffer left empty raises
     * no DREIF. What the host reads meanwhile is the fill (after the
     * shifter's leftover without BUFWR). In normal mode nothing is
     * written and the shifter sends back what it last received
     * (docs/avrdx/spi.md). Ends as exchange() does; the receive FIFO is
     * emptied at the start, with the select high.
     */
    template <typename T>
    static Run receive(std::span<T> in, uint32_t spins, uint8_t fill = 0xFF) {
        static_assert(run_byte<T>, "the AVR DA/DB SPI moves eight-bit characters (28.3): a "
                                   "run carries uint8_t");
        static_assert(available, "a client run needs the route's SCK, MOSI and SS pins, "
                                 "bonded on this package");
        const uint16_t len = static_cast<uint16_t>(in.size());
        if (len == 0) return {0, RunEnd::complete};
        return S::buffer_mode() ? serve_buffered<false>(nullptr, fill, in.data(), len, spins)
                                : serve_normal<false>(nullptr, in.data(), len, spins);
    }

    static void enable_interrupt(bool on) { S::enable_interrupt(on); }
    static void enable_rxc_interrupt(bool on) { S::enable_rxc_interrupt(on); }
    static void enable_txc_interrupt(bool on) { S::enable_txc_interrupt(on); }
    static void enable_dre_interrupt(bool on) { S::enable_dre_interrupt(on); }

    /// ISR bodies (SPIn_INT_vect), one per layout - the resource's, so
    /// the clear discipline is written once.
    [[gnu::always_inline]] static typename S::NormalIsr take_normal() { return S::take_normal(); }
    [[gnu::always_inline]] static uint8_t take_buffer() { return S::take_buffer(); }

    /// ClockUser: a client derives no rate from CLK_PER - the host does
    /// the clocking - but the FASTEST SCK it can follow is CLK_PER/6
    /// (errata clarification 3.7.3), so the peripheral clock is kept for
    /// that readback. Nothing is reprogrammed.
    static void rebase(uint32_t hz) { clk_per_hz_ = hz; }
    static uint32_t max_sck_hz() { return spi_max_client_sck_hz(clk_per_hz_); }
    /// Can this client follow a host running at `sck_hz`?
    static bool can_follow(uint32_t sck_hz) {
        return clk_per_hz_ != 0 && sck_hz <= spi_max_client_sck_hz(clk_per_hz_);
    }

    static void release() { S::release(); }

private:
    /// The one character type a run may carry: the beat is the element.
    template <typename T>
    static constexpr bool run_byte = std::is_same_v<T, uint8_t>;

    /// The select pad, read through its VPORT (an IN or an SBIC).
    [[gnu::always_inline]] static bool ss_high() {
        constexpr SpiPin sl = spi_pin(n, route, SpiSignal::ss);
        using Ss = Pin<sl.port, sl.pin>;
        return (Ss::vport().IN & Ss::mask) != 0;
    }

    /// Poll INTFLAGS until `flag` stands, and hand back the flags as that
    /// read found them. Zero when the host raised the select first (`how`
    /// closed) or the budget ran out (stalled). FOUR POLLS A COUNT: the
    /// select is asked once every four reads, so the read that sees the
    /// flag follows the one before it by a load and a skip; the budget is
    /// counted on an eight-bit counter and charged to `spins` 256 counts
    /// at a time, so the idle path between two groups of polls is an SBIC,
    /// a DEC and a branch and the 32-bit budget is touched once in 256.
    template <uint8_t flag>
    [[gnu::always_inline]] static uint8_t await(uint32_t& spins, RunEnd& how) {
        auto& s = S::regs();
        uint8_t c = 0;
        for (;;) {
            uint8_t f = s.INTFLAGS;
            if ((f & flag) != 0) return f;
            f = s.INTFLAGS;
            if ((f & flag) != 0) return f;
            f = s.INTFLAGS;
            if ((f & flag) != 0) return f;
            f = s.INTFLAGS;
            if ((f & flag) != 0) return f;
            if (ss_high()) {
                how = RunEnd::closed;
                return 0;
            }
            if (--c == 0) {
                if (spins <= 256u) {
                    spins = 0;
                    how = RunEnd::stalled;
                    return 0;
                }
                spins -= 256u;
            }
        }
    }

    /// The window opens: the select pad low, or a byte in (`mask`, a flag
    /// a transfer raises), whichever is seen first. False on the budget.
    static bool await_select(uint8_t mask, uint32_t& spins) {
        for (;;) {
            if (!ss_high()) return true;
            if ((S::regs().INTFLAGS & mask) != 0) return true;
            if (--spins == 0) return false;
        }
    }

    /// The buffer-mode preload, with the select high: the first answer,
    /// and under BUFWR - whose first write is copied into the shifter the
    /// cycle after it (28.3.2.2.2) - the second into the buffer. Returns
    /// how many were written (0: the budget ran out).
    template <bool has_out>
    [[gnu::always_inline]] static uint8_t preload_buffered(const uint8_t* out, uint8_t fill,
                                                           uint16_t len, bool bufwr,
                                                           uint32_t& spins) {
        auto& s = S::regs();
        s.DATA = has_out ? out[0] : fill;
        if (!bufwr || len < 2) return 1;
        while ((s.INTFLAGS & SPI_DREIF_bm) == 0) {
            if (--spins == 0) return 0;
        }
        s.DATA = has_out ? out[1] : fill;
        return 2;
    }

    /// The flags of a turn that came too late (the class's runs): TXCIF,
    /// the transmitter run dry, or BUFOVF, a byte lost behind a full FIFO.
    static constexpr uint8_t late_flags = SPI_TXCIF_bm | SPI_BUFOVF_bm;

    /// The steady turns of serve_buffered(), out of line so that its frame
    /// holds the loop's state alone and every value of a turn stays in a
    /// register: the next answer loaded before the poll, the flag seen, the
    /// late flags tested, the byte before the newest read and stored, the
    /// answer stored. Returns where the reads end; `how` is left complete
    /// when all `k` turns ran, and says why otherwise (late: the turn that
    /// saw the flags read nothing and wrote nothing).
    template <bool has_out>
    [[gnu::noinline]] static uint8_t* steady_turns(const uint8_t* p, uint8_t fill, uint8_t* q,
                                                   uint16_t k, uint32_t& spins, RunEnd& how) {
        auto& s = S::regs();
        do {
            const uint8_t next = has_out ? *p++ : fill;
            const uint8_t f = await<SPI_DREIF_bm>(spins, how);
            if (f == 0) break;
            if ((f & late_flags) != 0) {
                how = RunEnd::late;
                break;
            }
            *q++ = s.DATA;
            s.DATA = next;
        } while (--k != 0);
        return q;
    }

    /// exchange() and receive() in BUFFER MODE, read by count.
    ///
    /// Event t is the end of transfer t. It raises DREIF while the buffer
    /// holds an answer - for every t < len without BUFWR, for t < len - 1
    /// under it - and TXCIF for the last one under BUFWR, the shifter's
    /// byte being the run's last. Event 0 writes the next answer; every
    /// later one READS byte t - 1, the older of the two newest, AND THEN
    /// writes the next answer, and the event after the last write reads
    /// under BUFWR; the last two bytes are read once the last event is in.
    ///
    /// THE READ COMES FIRST. A turn's two deadlines are the end of the next
    /// transfer: the answer must be in the buffer by then (or the
    /// transmitter runs dry and TXCIF rises), and the byte before the newest
    /// must be out of the FIFO, which is full until it is (or the arriving
    /// byte, held in the shifter, is lost when the transfer after starts -
    /// silently: BUFOVF is not always raised for it). A turn that read
    /// after writing would meet the transmitter's deadline and miss the
    /// receiver's in a band of lateness a few cycles wide, and lose a byte
    /// that no flag reports; reading first, a turn that misses the
    /// receiver's deadline has missed the transmitter's too, and the next
    /// event shows TXCIF (or BUFOVF). The run then ends `late` on the bytes
    /// read before that event: an in-order stream with no gap
    /// (docs/avrdx/spi.md, the client's sweep). The read is the flag's
    /// load and a test after it - past RXCIF's one cycle (28.3.2.2.2) -,
    /// and the FIFO is never emptied while a byte may enter it.
    template <bool has_out>
    static Run serve_buffered(const uint8_t* out, uint8_t fill, uint8_t* in, uint16_t len,
                              uint32_t spins) {
        auto& s = S::regs();
        S::flush_receive();
        if ((s.INTFLAGS & SPI_DREIF_bm) == 0) return {0, RunEnd::stalled};
        const bool bufwr = S::buffer_wait();
        const uint8_t pre = preload_buffered<has_out>(out, fill, len, bufwr, spins);
        if (pre == 0) return {0, RunEnd::stalled};
        // Shifter (and buffer) loaded: no empty moment can raise TXCIF
        // until the transmitter runs dry - the run's last transfer, or a
        // late turn.
        s.INTFLAGS = SPI_TXCIF_bm;
        if (!await_select(SPI_RXCIF_bm, spins)) return {0, RunEnd::stalled};
        const uint8_t* p = has_out ? out + pre : nullptr;
        uint8_t* q = in;
        uint8_t* known = in;
        RunEnd how = RunEnd::complete;
        uint16_t writes = static_cast<uint16_t>(len - pre);   // answers still to write
        const uint16_t dreifs = bufwr ? static_cast<uint16_t>(len - 1u) : len;
        uint8_t mask = SPI_DREIF_bm;
        uint8_t f = 0;
        if (dreifs != 0) {
            // Event 0: the next answer, nothing to read yet.
            f = await<SPI_DREIF_bm>(spins, how);
            if (f == 0) goto cut;
            if ((f & late_flags) != 0) goto late;
            known = in + 1;
            if (writes != 0) {
                s.DATA = has_out ? *p++ : fill;
                --writes;
            }
            // The steady state: the byte before the newest read, an answer
            // written - one event, one load, one store.
            const uint16_t steady = writes;
            if (steady != 0) {
                q = steady_turns<has_out>(p, fill, q, steady, spins, how);
                known = q + 1;
                if (how == RunEnd::late) {
                    known = q;
                    goto tail;
                }
                if (how != RunEnd::complete) goto cut;
            }
            // The event after the last write, when there is one: under
            // BUFWR it is not the run's last and reads; without BUFWR it
            // is, and the tail reads.
            if (dreifs != static_cast<uint16_t>(steady + 1u)) {
                f = await<SPI_DREIF_bm>(spins, how);
                if (f == 0) goto cut;
                if ((f & late_flags) != 0) {
                    known = q;
                    goto late;
                }
                known = q + 2;
                if (bufwr) {
                    *q++ = s.DATA;
                }
            }
        }
        if (bufwr) {
            mask = SPI_TXCIF_bm;
            f = await<SPI_TXCIF_bm>(spins, how);
            if (f == 0) goto cut;
        }
        known = in + len;
        goto tail;
    cut:
        // After the close nothing moves: a flag that rose before it is a
        // transfer that ended inside the window.
        if (how == RunEnd::closed && (s.INTFLAGS & mask) != 0 && known < in + len) ++known;
        goto tail;
    late:
        how = RunEnd::late;
    tail:
        // What the count says has arrived, in order. The last read may meet
        // a byte the host clocks past the run: it returns its own byte all
        // the same, and the next run's flush takes what it leaves.
        while (q < known) *q++ = s.DATA;
        const uint16_t got = static_cast<uint16_t>(q - in);
        if (how == RunEnd::closed && got == len) how = RunEnd::complete;
        return {got, how};
    }

    /// exchange() and receive() in NORMAL MODE: one byte in flight, IF a
    /// transfer (28.3.2.2.1). The answer is written first, in the host's
    /// gap, then the byte read; the INTFLAGS read that saw IF and the DATA
    /// access clear it (28.5.4). A write that met the next transfer was
    /// ignored and raised WRCOL: `late` - and the byte that turn reads is
    /// NOT counted: IF is one flag, not a count, so a turn that came a
    /// transfer late may find DATA holding a later byte than the one IF
    /// announced (measured: test_avr_spi letter v, a handler timed into the
    /// run; docs/avrdx/spi.md). The read is still made - it is the clear.
    /// NORMAL MODE HAS NO COUNT: a turn delayed past a whole transfer and
    /// its gap writes in the next gap, raises nothing, and the byte it
    /// missed is gone with no flag to say so (the same letter) - a run
    /// that must know is a buffer-mode run.
    template <bool has_out>
    static Run serve_normal(const uint8_t* out, uint8_t* in, uint16_t len, uint32_t spins) {
        auto& s = S::regs();
        (void)s.INTFLAGS;
        (void)s.DATA;                      // IF and WRCOL cleared, the receiver read out
        uint16_t w = 0;
        if constexpr (has_out) {
            s.DATA = out[0];               // the select high: the shifter
            w = 1;
        }
        uint16_t r = 0;
        RunEnd how = RunEnd::complete;
        if (!await_select(SPI_IF_bm, spins)) return {0, RunEnd::stalled};
        for (;;) {
            if (await<SPI_IF_bm>(spins, how) == 0) {
                if (how == RunEnd::closed && (s.INTFLAGS & SPI_IF_bm) != 0) in[r++] = s.DATA;
                break;
            }
            if constexpr (has_out) {
                if (w < len) {
                    s.DATA = out[w];
                    ++w;
                    if ((s.INTFLAGS & SPI_WRCOL_bm) != 0) how = RunEnd::late;
                }
            }
            const uint8_t v = s.DATA;
            if (how == RunEnd::late) break;
            in[r++] = v;
            if (r == len) break;
        }
        if (how == RunEnd::closed && r == len) how = RunEnd::complete;
        return {r, how};
    }

    /// answer() in BUFFER MODE: the preload, then an answer on each DREIF;
    /// TXCIF before the last is loaded is a turn that missed its transfer.
    static Run answer_buffered(const uint8_t* out, uint16_t len, uint32_t spins) {
        auto& s = S::regs();
        if ((s.INTFLAGS & SPI_DREIF_bm) == 0) return {0, RunEnd::stalled};
        uint16_t w = preload_buffered<true>(out, 0, len, S::buffer_wait(), spins);
        RunEnd how = RunEnd::closed;
        if (w == 0) {
            how = RunEnd::stalled;
        } else {
            s.INTFLAGS = SPI_TXCIF_bm;
            if (!await_select(SPI_RXCIF_bm, spins)) {
                how = RunEnd::stalled;
            } else {
                for (; w < len; ++w) {
                    const uint8_t f = await<SPI_DREIF_bm>(spins, how);
                    if (f == 0) break;
                    if ((f & SPI_TXCIF_bm) != 0) {
                        how = RunEnd::late;
                        break;
                    }
                    s.DATA = out[w];
                }
                if (w == len) {
                    while (!ss_high()) {   // every answer loaded: the host's close ends it
                        if (--spins == 0) {
                            how = RunEnd::stalled;
                            break;
                        }
                    }
                }
            }
        }
        S::flush_receive();
        return {w, how};
    }

    /// answer() in NORMAL MODE: the first loaded with the select high, the
    /// rest on IF, each in the host's gap; WRCOL behind a write is `late`.
    static Run answer_normal(const uint8_t* out, uint16_t len, uint32_t spins) {
        auto& s = S::regs();
        (void)s.INTFLAGS;
        (void)s.DATA;
        s.DATA = out[0];
        uint16_t w = 1;
        RunEnd how = RunEnd::closed;
        if (!await_select(SPI_IF_bm, spins)) {
            how = RunEnd::stalled;
        } else {
            for (; w < len; ++w) {
                if (await<SPI_IF_bm>(spins, how) == 0) break;
                s.DATA = out[w];
                if ((s.INTFLAGS & SPI_WRCOL_bm) != 0) {
                    how = RunEnd::late;   // ignored: not loaded
                    break;
                }
            }
            if (w == len && how == RunEnd::closed) {
                while (!ss_high()) {
                    if (--spins == 0) {
                        how = RunEnd::stalled;
                        break;
                    }
                }
            }
        }
        (void)s.INTFLAGS;
        (void)s.DATA;                      // IF and WRCOL cleared for the next run
        return {w, how};
    }

    static inline uint32_t clk_per_hz_ = 0;
};

static_assert(ClockUser<SpiHost<0>>);
static_assert(ClockUser<SpiClient<0>>);

} // namespace brio
