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
 *                         transactions in one chip-select window, a
 *                         per-byte ISR pump or a polled loop, CS and DC
 *                         owned by the engine. Driven by
 *                         util/spi_bus.hpp (arbitration and replies).
 *    SpiClient<n, route>  the client side: selected(), preload/exchange,
 *                         the buffer-mode variants, the ISR bodies.
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
 *    it has no event users and no DBGCTRL/RUNSTDBY of its own.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <type_traits>
#include <avr/io.h>

#include "avrdx/delay.hpp"
#include "avrdx/evsys.hpp"
#include "avrdx/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// ---- routes and pins (17.5.8, 28.2.2) ---------------------------------------

/// SPI pin routing (PORTMUX.SPIROUTEA). The values ARE the device
/// header's own group values, `none` included.
enum class SpiRoute : uint8_t {
    def = 0,
    alt1 = 1,
    alt2 = 2,
    none = 3,
};
static_assert(static_cast<uint8_t>(SpiRoute::def) == PORTMUX_SPI0_DEFAULT_gv &&
              static_cast<uint8_t>(SpiRoute::none) == PORTMUX_SPI0_NONE_gv,
              "the route codes must be the device header's group values");

/// The pin count of this package, read off the device header's own
/// tiers (PORTG is 64-pin, PORTB/PORTE are 32-pin and up... on this
/// family PORTB and PORTE appear together at 48 pins, and TWI1 marks
/// the 32-pin step). Only the >= 48 and >= 64 steps change an SPI route
/// table; the smaller ones are kept for the doc's sake.
#if defined(PORTG)
inline constexpr uint8_t spi_package_pins = 64;
#elif defined(PORTE)
inline constexpr uint8_t spi_package_pins = 48;
#elif defined(TWI1)
inline constexpr uint8_t spi_package_pins = 32;
#else
inline constexpr uint8_t spi_package_pins = 28;
#endif

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
    if (n > 1) return false;
    if (r == SpiRoute::none) return true;
    if (r == SpiRoute::def) return true;                  // both instances, every package
    if (r == SpiRoute::alt1) return spi_package_pins >= 48;
    // ALT2: SPI0 on PORTG (64-pin only); SPI1 on PORTB - listed by the
    // 48-pin headers without SCK/SS and declared non-functional there
    // by DB errata 2.11.1, so 64 pins for both.
    if (spi_package_pins < 64) return false;
    return port_exists(spi_port_letter(n, r));
}

/// The position of one signal of one route on THIS package.
constexpr SpiPin spi_pin(uint8_t n, SpiRoute r, SpiSignal s) {
    if (n > 1 || r == SpiRoute::none) return {};
    const char port = spi_port_letter(n, r);
    if (port == '?' || !port_exists(port)) return {};
    const uint8_t pin = static_cast<uint8_t>(spi_first_pin(n, r) + static_cast<uint8_t>(s));
    // SPI1 ALT2 on a 48-pin package stops at PB5: the route as a whole
    // is refused above, but the table stays honest about why.
    if (n == 1 && r == SpiRoute::alt2 && spi_package_pins < 64 &&
        static_cast<uint8_t>(s) >= 2) {
        return {port, pin, false};
    }
    return {port, pin, true};
}

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

/// The CTRLA bits (PRESC and CLK2X) that produce this rate.
constexpr uint8_t spi_presc_bits(SpiClock c) {
    switch (c) {
        case SpiClock::div2: return static_cast<uint8_t>(SPI_PRESC_DIV4_gc | SPI_CLK2X_bm);
        case SpiClock::div4: return static_cast<uint8_t>(SPI_PRESC_DIV4_gc);
        case SpiClock::div8: return static_cast<uint8_t>(SPI_PRESC_DIV16_gc | SPI_CLK2X_bm);
        case SpiClock::div16: return static_cast<uint8_t>(SPI_PRESC_DIV16_gc);
        case SpiClock::div32: return static_cast<uint8_t>(SPI_PRESC_DIV64_gc | SPI_CLK2X_bm);
        case SpiClock::div64: return static_cast<uint8_t>(SPI_PRESC_DIV64_gc);
        default: return static_cast<uint8_t>(SPI_PRESC_DIV128_gc);
    }
}

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
    if (n > 1) return false;
    if (!spi_route_exists(n, c.route)) return false;
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
    static_assert(n <= 1, "AVR DA/DB carry SPI0 and SPI1 on every package");

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
        static_assert(spi_route_exists(n, cfg.route),
                      "this package does not bond this SPI route (28/32 pins have "
                      "DEFAULT only; SPI1 ALT2 is refused on 48-pin parts: DB errata "
                      "2.11.1, and no SCK position is bonded there anyway)");
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
        if constexpr (n == 0) return SPI0;
        else return SPI1;
    }

    /// PORTMUX position of this instance's two route bits.
    static constexpr uint8_t route_gp() {
        if constexpr (n == 0) return PORTMUX_SPI0_gp;
        else return PORTMUX_SPI1_gp;
    }

    static void write_route(SpiRoute r) {
        constexpr uint8_t gp = route_gp();
        constexpr uint8_t gm = static_cast<uint8_t>(0x03u << gp);
        PORTMUX.SPIROUTEA = static_cast<uint8_t>(
            (PORTMUX.SPIROUTEA & ~gm) | static_cast<uint8_t>(static_cast<uint8_t>(r) << gp));
    }

    /// The route bits as they read back.
    static SpiRoute routed() {
        constexpr uint8_t gp = route_gp();
        constexpr uint8_t gm = static_cast<uint8_t>(0x03u << gp);
        return static_cast<SpiRoute>((PORTMUX.SPIROUTEA & gm) >> gp);
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
 * byte pump under the SPI interrupt (no DMA on AVR Dx: one interrupt per
 * byte is the honest price).
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
 * Two completion styles, chosen per request by the `polled` flag: the
 * per-byte ISR pump (default) and the synchronous polled loop for bulk
 * transfers at fast clocks - see the flag's comment for the tradeoff.
 *
 * ISR wiring (app glue, as usual):
 *   ISR(SPI0_INT_vect) {
 *       if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{brio::spi_ok}); }
 *   }
 */
template <uint8_t n, SpiRoute route = SpiRoute::def>
class SpiHost {
    using S = Spi<n>;
    static_assert(spi_route_exists(n, route),
                  "this package does not bond this SPI route (28/32 pins have DEFAULT "
                  "only; SPI1 ALT2 is refused on 48-pin parts: DB errata 2.11.1)");

public:
    SpiHost() = delete;

    using Resource = S;
    static constexpr SpiRoute pin_route = route;

    /// Does this package bond everything a host on this route needs?
    static constexpr bool available =
        spi_route_exists(n, route) &&
        (route == SpiRoute::none || spi_pin(n, route, SpiSignal::sck).bonded);

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
        // at 6 MHz, XPT2046 capped at ~2.5 MHz, ...); the engine
        // reprograms the peripheral at each start(), which costs two
        // register writes between transactions and nothing per byte.
        // The rate is a DIVISION of CLK_PER, so it follows a clock
        // change by itself; max_sck_hz() is the ceiling that clamps it.
        SpiClock clock = SpiClock::div16;
        SpiMode mode = SpiMode::mode0;
        // Completion style, also the client's call: false = per-byte
        // ISR pump (the kernel keeps running between bytes - right for
        // slow clocks and short transfers); true = POLLED inside
        // start(), completing synchronously. At fast clocks polling
        // wins on every axis: a byte at div4 flies in 32 CPU cycles
        // while an ISR entry alone costs more - the pump caps the bus
        // near 27% and floods the CPU, the polled loop runs it near
        // wire speed. The price is that THIS dispatch blocks for the
        // whole transfer (bounded, chosen here); global interrupts
        // stay enabled throughout - only the SPI's own IE is silenced.
        bool polled = false;
        // Chip-select setup: microseconds the engine waits between
        // asserting CS and the first SCK edge. Most devices need tens of
        // ns (the ~1.5 us the code path takes anyway). A device waking
        // from shutdown on CS may need more: the MCP3550 datasheet
        // specifies tRDY <= 50 ns but only says "an internal power-up
        // delay must be observed" when exiting Shutdown (DS20001950F
        // 5.2) - MEASURED: SDO drives ~4 us after CS falls, and a frame
        // clocked 1.5 us after CS is lost (0x7FFFFF), 3.5 us is enough,
        // dac_adc uses 10. Spent spinning in start(), main context,
        // bounded by this byte.
        uint8_t cs_setup_us = 0;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    /**
     * Host, MSB first, SSD set (the SS position stays free for a GPIO -
     * chip selects are the engine's, not the peripheral's). Call after
     * clock init, before sei(). CS/DC pins are configured by their
     * owners (the device clients), not here; clock and mode travel
     * per-request. `clock` is the app's brio::Clock tag: the per-request
     * cs_setup_us delay is timed from Clock::hz.
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
            cpol_ = false;
            return S::init({.route = route, .role = SpiRole::host, .mode = SpiMode::mode0,
                            .clock = ceiling_ ? *ceiling_ : SpiClock::div16,
                            .client_select_disable = true});
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

    /// Begin a transaction (called by SpiBus from main context).
    /// Returns true when the transaction completed synchronously
    /// (polled requests, and the degenerate zero-length one); false
    /// when it runs on the ISR and a TransferDone will follow.
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        in_cmd_ = (r.cmd_len > 0);
        if (total_len() == 0) {
            return true;  // nothing to move: complete on the spot
        }
        apply_mode(r.mode, clamp(r.clock));
        if (in_cmd_) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();                      // assert, active low
        if (r.cs_setup_us != 0) {
            delay_us_runtime(cycles_per_us_, r.cs_setup_us);
        }
        if (!r.polled) {
            S::enable_interrupt(true);
            S::write(first_byte());        // the ISR pumps the rest
            return false;
        }
        // Polled pump: silence the SPI's own interrupt (the bound ISR
        // would steal the bytes) - global interrupts STAY ENABLED, so
        // UART/PIT/anything else preempt this loop freely. The last
        // xfer() leaves INTFLAGS clear, so re-enabling IE is safe.
        S::enable_interrupt(false);
        // The loans are VIEWS: .get() hands out the raw pointer the
        // loops index (Borrowed is not a container).
        for (uint8_t i = 0; i < r.cmd_len; ++i) {
            xfer(r.cmd.get()[i]);
        }
        r.dc.set();                        // data phase (no-op if len == 0)
        // Shape-specialized loops: the per-byte budget at div4 is 32
        // cycles, so hoisting the tx/rx null checks out of the loop is
        // not cosmetics - it is most of the headroom.
        if (r.rx.get() == nullptr && r.tx.get() != nullptr) {   // bulk write
            const uint8_t* p = r.tx.get();
            for (uint16_t k = r.len; k != 0; --k) {
                xfer(*p++);
            }
        } else if (r.rx.get() != nullptr && r.tx.get() == nullptr) {  // bulk read
            uint8_t* p = r.rx.get();
            for (uint16_t k = r.len; k != 0; --k) {
                *p++ = xfer(0xFF);
            }
        } else {                                           // full duplex / none
            for (uint16_t i = 0; i < r.len; ++i) {
                const uint8_t in = xfer((r.tx.get() != nullptr) ? r.tx.get()[i] : 0xFF);
                if (r.rx.get() != nullptr) {
                    r.rx.get()[i] = in;
                }
            }
        }
        r.cs.set();                        // release: transaction done
        S::enable_interrupt(true);
        return true;
    }

    /**
     * @brief SPI interrupt body - call from ISR(SPIn_INT_vect).
     * @return true when the transaction just completed (CS released):
     * the edge on which the glue posts TransferDone to the bus AO.
     */
    [[gnu::always_inline]] static bool isr() {
        const uint8_t in = S::take_normal().data;   // INTFLAGS then DATA: the IF clear sequence

        if (!in_cmd_ && req_.rx.get() != nullptr) {
            req_.rx.get()[pos_] = in;
        }
        ++pos_;

        if (in_cmd_ && pos_ >= req_.cmd_len) {
            in_cmd_ = false;
            pos_ = 0;
            req_.dc.set();                 // command phase over
        }
        if (!in_cmd_ && pos_ >= req_.len) {
            req_.cs.set();                 // release: transaction done
            return true;
        }
        S::write(next_byte());
        return false;
    }

    /// The work-around for a wedged host: a demotion mid-transfer (SS
    /// driven low with SSD clear stops the ISR pump dead, 28.3.2.1.3 -
    /// bench-measured on the shared-SS letter) or a lost completion.
    /// The interrupt is silenced and its flag cleared by the
    /// INTFLAGS-then-DATA sequence, the Host role the hardware dropped
    /// is re-armed, the select window closed; start() reprograms mode
    /// and clock per request, so nothing else needs saving. The verb a
    /// timed SpiBus calls (util/bus_master.hpp). COMPILE-VERIFIED
    /// against the bench suites' engine only: the timed path itself has
    /// not run on AVR silicon yet (docs/avrdx/spi.md).
    static void recover() {
        if constexpr (available) {
            S::enable_interrupt(false);
            (void)S::take_normal();
            if (S::demoted()) {
                S::restore_host();
            }
            req_.cs.set();
            in_cmd_ = false;
        }
    }

    /// Hand the route's pins back (the resource's teardown).
    static void release() { S::release(); }

private:
    static SpiClock clamp(SpiClock c) {
        if (!ceiling_) return c;
        return spi_division(c) < spi_division(*ceiling_) ? *ceiling_ : c;
    }

    static uint16_t total_len() {
        return static_cast<uint16_t>(req_.cmd_len) + req_.len;
    }

    /// SCK must already sit at the new mode's idle level (CPOL) when CS
    /// falls: devices that latch their SPI mode from SCK at the CS edge
    /// (MCP3550: mode 0,0 vs 1,1) otherwise start the transaction in the
    /// wrong mode. The AVR SPI updates the SCK output level when it is
    /// ENABLED and at every transfer - NOT on a CTRLB write while it is
    /// enabled (seen on the analyzer: SCK still low 10 us after CS fell
    /// on the first mode-3 request after init; a known AVR quirk). So a
    /// CPOL change is applied with the peripheral disabled: preset the
    /// SCK pin's PORT.OUT to the new idle level (what the pin shows while
    /// the SPI is off - no glitch), disable, write the mode, re-enable.
    /// Three register writes, no clock edges on the bus, only when the
    /// polarity changes between transactions.
    static void apply_mode(SpiMode mode, SpiClock clock) {
        const bool cpol = spi_cpol(mode);
        if (cpol != cpol_) {
            cpol_ = cpol;
            if constexpr (route != SpiRoute::none) {
                constexpr SpiPin sc = spi_pin(n, route, SpiSignal::sck);
                if constexpr (sc.bonded) {
                    volatile PORT_t& port = port_by_letter(sc.port);
                    if (cpol) port.OUTSET = static_cast<uint8_t>(1u << sc.pin);
                    else port.OUTCLR = static_cast<uint8_t>(1u << sc.pin);
                }
            }
            S::enable(false);
        }
        S::mode(mode);
        S::clock(clock);
        S::enable(true);
    }

    /// One polled byte: write, spin on the completion flag (~1 byte
    /// time), read back. Reading INTFLAGS (IF set) then DATA is the IF
    /// clear sequence.
    static uint8_t xfer(uint8_t out) {
        S::write(out);
        while (!S::if_flag()) {}
        return S::read();
    }

    static uint8_t first_byte() { return in_cmd_ ? req_.cmd.get()[0] : data_byte(0); }

    static uint8_t next_byte() {
        return in_cmd_ ? req_.cmd.get()[pos_] : data_byte(pos_);
    }

    static uint8_t data_byte(uint16_t i) {
        return (req_.tx.get() != nullptr) ? req_.tx.get()[i] : 0xFF;
    }

    static inline Request req_{};
    static inline uint16_t pos_ = 0;
    static inline bool in_cmd_ = false;
    static inline bool cpol_ = false;           // init() leaves mode 0: SCK low
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
 */
template <uint8_t n, SpiRoute route = SpiRoute::def>
class SpiClient {
    using S = Spi<n>;
    static_assert(spi_route_exists(n, route),
                  "this package does not bond this SPI route (28/32 pins have DEFAULT "
                  "only; SPI1 ALT2 is refused on 48-pin parts: DB errata 2.11.1)");

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
    /// when this package cannot bond the route's pins.
    static bool init(Options o = {}) {
        if constexpr (!available) {
            (void)o;
            return false;
        } else {
            return S::init({.route = route, .role = SpiRole::client, .mode = o.mode,
                            .lsb_first = o.lsb_first,
                            .client_select_disable = true,   // SSD is a host bit; keep it out of the way
                            .buffer_mode = o.buffer_mode,
                            .buffer_wait = o.buffer_wait,
                            .drive_miso = o.drive_miso});
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
    static inline uint32_t clk_per_hz_ = 0;
};

static_assert(ClockUser<SpiHost<0>>);
static_assert(ClockUser<SpiClient<0>>);

} // namespace brio
