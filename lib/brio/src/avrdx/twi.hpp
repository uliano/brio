/*
 * twi.hpp
 *
 * The AVR DA/DB Two-Wire Interface (DS40002247B ch. 29) in two strata,
 * as docs/avrdx/twi.md describes:
 *
 *  RESOURCE - Twi<n>: the typed view of one instance. A config struct
 *  owns the whole configuration (route, the shared CTRLA knobs, the
 *  host half, the client half, the dual-mode client); init<cfg>()
 *  folds it and refuses at compile time what this package cannot bond,
 *  init(cfg) computes it at run time and returns false instead. Then
 *  the verbs of the chapter's register description: CTRLA, DUALCTRL,
 *  DBGCTRL, MCTRLA/MCTRLB/MSTATUS/MBAUD/MADDR/MDATA on the host side,
 *  SCTRLA/SCTRLB/SSTATUS/SADDR/SDATA/SADDRMASK on the client side, the
 *  two ISR bodies (one per vector), and release().
 *
 *  TASKS - what an application names:
 *    TwiHost<n, route>    the transfer ENGINE: one Request = ONE bus
 *                         tenure (write / read / write-then-read with a
 *                         repeated START / probe), the byte pump under
 *                         the host interrupt. Driven by util/i2c_bus.hpp
 *                         (arbitration and replies).
 *    TwiClient<n, route, on_dual_pins>
 *                         the other end: the address-match space, the
 *                         S1..S4 protocol surface as verbs, a polled
 *                         surface and the ISR body.
 *  Host and client of ONE instance may run together - COMBINED mode
 *  (same pins) or DUAL mode (the client on the route's second pin pair).
 *  That is a first-class configuration here, not an accident.
 *
 * Facts that shape the code (29.3, 29.5, errata DS80000915F 2.15.1/
 * 2.15.2 and DS80000882C 2.14.1/2.14.2/2.14.3):
 *  - TWI0 exists on every package; TWI1 appears at 32 pins. Unlike SPI
 *    and USART the TWI has NO pinless route: every PORTMUX code names a
 *    real SDA/SCL pair, so an instance always costs two pins. The route
 *    table below is the device headers' own route enums, per package;
 *  - each route also names a DUAL pair - where the client sits when
 *    DUALCTRL.ENABLE is set. Several routes bond the main pair but not
 *    the dual one (TWI0 ALT1/ALT2 below 48 pins, TWI1 DEFAULT at 32
 *    pins, TWI1 ALT1/ALT2 below 64 pins): the main pair stays usable and
 *    only the dual client is refused;
 *  - ERRATA AS CODE, all three of them:
 *      * DB 2.15.1 (rev. A4/A5) / DA 2.14.1 (every rev.): with the TWI
 *        enabled the pin override covers the DRIVER but not the VALUE -
 *        a PORTx.OUT bit left at '1' on SDA or SCL holds that line high
 *        and no transaction can start. Both init paths clear the OUT
 *        bits of every pin they route, unconditionally: it is one store
 *        per pin and it costs nothing on the revisions that do not need
 *        it;
 *      * DB 2.15.2 / DA 2.14.3 (every rev.): MCTRLB.FLUSH can leave the
 *        host stuck in the Unknown bus state. This resource does not
 *        expose FLUSH AT ALL - recover() is the documented work-around
 *        (an ENABLE cycle on the host) followed by force_idle();
 *  - recover() and unstick() fix DIFFERENT things and neither replaces
 *    the other: recover() puts this PERIPHERAL back into a known state,
 *    unstick() bit-bangs the WIRE free of a client that is holding SDA
 *    low (the classic nine-clocks-and-a-STOP remedy). The policy above
 *    them - noticing a stuck transaction at all - is the bus AO's and is
 *    not built (design/i2c-bus.md);
 *      * DA 2.14.2 (every DA rev., DA only): the SDAHOLD 50 ns and
 *        300 ns SELECTIONS are swapped in the silicon. TwiSdaHold names
 *        TRUE nanoseconds and twi_sdahold_bits() swaps the encoding on
 *        the DA - the family is told apart by MVIO, which only the DB
 *        has;
 *  - the baud arithmetic is the chapter's own three steps (29.3.2.2.1):
 *    equation 29-3, then equation 29-4 for tLOW, then equation 29-5
 *    when that tLOW came out below the mode's floor (4700 / 1300 /
 *    500 ns). The bus's RISE and FALL times are arguments, because that
 *    is what the equations take: the default charges the specification's
 *    maxima for the mode and a bus that declares its measured edges gets
 *    the speed back. Charging nothing for the FALL is the one unsafe
 *    choice - the SCL low time the pins really show is the register's
 *    BAUD + 5 clocks MINUS the fall, so it would land below the floor by
 *    exactly tOF (bench: 125 ns on slew-limited pads). MBAUD "must be
 *    written while the host is disabled" (29.5.7), so set_baud()
 *    performs the ENABLE cycle and re-declares the bus idle;
 *  - FMPEN is what Fast-mode Plus means to the PINS (x10 drive instead
 *    of a slew limit, 29.3.3.1); the divider knows nothing about it. So
 *    TwiSpeed::fast_plus_1m and CTRLA.FMPEN are checked against each
 *    other by twi_config_valid: asking for 1 MHz with FMPEN off is
 *    refused rather than silently run on slew-limited pads;
 *  - CLK_PER must be at least four times f_SCL for the bus-error
 *    detector to work at all (29.5.6 BUSERR) and for the client's Stop
 *    interrupt (PIEN, 29.5.10). clock_ok() is the readback of that
 *    condition; the tasks report it, they do not enforce it (a slower
 *    main clock is legal, it only blinds those two features);
 *  - two vectors, two ISR bodies: TWIn_TWIM_vect (host: RIF/WIF) and
 *    TWIn_TWIS_vect (client: DIF/APIF). Neither body clears anything -
 *    the host's flags clear when the handler writes MADDR/MDATA/MCMD
 *    and the client's when it writes SCMD or touches SDATA (29.5.6,
 *    29.5.12), so a body that cleared them would eat the state its
 *    caller has to read.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <type_traits>
#include <avr/io.h>

#include "avrdx/delay.hpp"
#include "avrdx/pin.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

// ---- routes and pins (17.5.9, 29.2.2) ---------------------------------------

/// TWI pin routing (PORTMUX.TWIROUTEA). The values ARE the device
/// header's own group values. There is no "none" code: the TWI cannot
/// run pinless.
enum class TwiRoute : uint8_t {
    def = 0,
    alt1 = 1,
    alt2 = 2,
};
static_assert(static_cast<uint8_t>(TwiRoute::def) == PORTMUX_TWI0_DEFAULT_gv &&
              static_cast<uint8_t>(TwiRoute::alt1) == PORTMUX_TWI0_ALT1_gv &&
              static_cast<uint8_t>(TwiRoute::alt2) == PORTMUX_TWI0_ALT2_gv,
              "the route codes must be the device header's group values");

/// The pin count of this package, read off the device header's own
/// tiers (the same ladder spi.hpp/usart.hpp use).
#if defined(PORTG)
inline constexpr uint8_t twi_package_pins = 64;
#elif defined(PORTE)
inline constexpr uint8_t twi_package_pins = 48;
#elif defined(TWI1)
inline constexpr uint8_t twi_package_pins = 32;
#else
inline constexpr uint8_t twi_package_pins = 28;
#endif

/// How many TWI instances this package carries: TWI1 arrives with the
/// 32-pin step.
#if defined(TWI1)
inline constexpr uint8_t twi_instance_count = 2;
#else
inline constexpr uint8_t twi_instance_count = 1;
#endif

/// The two signals of a route.
enum class TwiSignal : uint8_t { sda = 0, scl = 1 };

/// Where one signal of one route sits, and whether THIS package bonds
/// it out. `bonded == false` means the silicon has the function but no
/// pin for it here.
struct TwiPin {
    char port = '?';
    uint8_t pin = 0;
    bool bonded = false;
    constexpr explicit operator bool() const { return bonded; }
};

/// Whether this package offers a route at all (the device headers'
/// route enums):
///   TWI0  DEFAULT / ALT1 / ALT2 on every package;
///   TWI1  DEFAULT / ALT1 from 32 pins up, ALT2 from 48 pins up.
constexpr bool twi_route_exists(uint8_t n, TwiRoute r) {
    if (n >= twi_instance_count) return false;
    if (n == 0) return true;
    if (r == TwiRoute::alt2) return twi_package_pins >= 48;   // PB2/PB3
    return true;                                             // PF2/PF3
}

/// The MAIN pin pair of a route - where the host, and a client that is
/// not in Dual mode, sit:
///   TWI0  DEFAULT PA2/PA3   ALT1 PA2/PA3   ALT2 PC2/PC3
///   TWI1  DEFAULT PF2/PF3   ALT1 PF2/PF3   ALT2 PB2/PB3
constexpr TwiPin twi_pin(uint8_t n, TwiRoute r, TwiSignal s) {
    if (!twi_route_exists(n, r)) return {};
    const uint8_t off = static_cast<uint8_t>(s);
    if (n == 0) {
        if (r == TwiRoute::alt2) return {'C', static_cast<uint8_t>(2 + off), true};
        return {'A', static_cast<uint8_t>(2 + off), true};
    }
    if (r == TwiRoute::alt2) {
        return {'B', static_cast<uint8_t>(2 + off), port_exists('B')};
    }
    return {'F', static_cast<uint8_t>(2 + off), true};
}

/// The DUAL pin pair of a route - where the client sits when
/// DUALCTRL.ENABLE is set (29.3.3.4). Several routes have none on the
/// smaller packages, which is what the `bonded` flag says:
///   TWI0  DEFAULT PC2/PC3 (every package)
///         ALT1/ALT2 PC6/PC7 (48 pins and up)
///   TWI1  DEFAULT PB2/PB3 (48 pins and up)
///         ALT1/ALT2 PB6/PB7 (64 pins)
constexpr TwiPin twi_dual_pin(uint8_t n, TwiRoute r, TwiSignal s) {
    if (!twi_route_exists(n, r)) return {};
    const uint8_t off = static_cast<uint8_t>(s);
    if (n == 0) {
        if (r == TwiRoute::def) return {'C', static_cast<uint8_t>(2 + off), true};
        return {'C', static_cast<uint8_t>(6 + off), twi_package_pins >= 48};
    }
    if (r == TwiRoute::def) {
        return {'B', static_cast<uint8_t>(2 + off), twi_package_pins >= 48};
    }
    return {'B', static_cast<uint8_t>(6 + off), twi_package_pins >= 64};
}

/// Does this package bond BOTH dual pins of this route? (What a Dual
/// mode client needs.)
constexpr bool twi_has_dual(uint8_t n, TwiRoute r) {
    return twi_dual_pin(n, r, TwiSignal::sda).bonded &&
           twi_dual_pin(n, r, TwiSignal::scl).bonded;
}

// ---- the knobs (29.5.1 - 29.5.5, 29.5.10 - 29.5.11) -------------------------

/// CTRLA/DUALCTRL.INPUTLVL: the input voltage transition level.
enum class TwiInputLevel : uint8_t { i2c = 0, smbus = 1 };

/// CTRLA.SDASETUP: how many peripheral clocks the CLIENT stretches SCL
/// to guarantee the setup time of its SDA output (29.5.1; the DA
/// clarification 3.2 restates it as the clock-hold time).
enum class TwiSdaSetup : uint8_t { cycles4 = 0, cycles8 = 1 };

/// CTRLA/DUALCTRL.SDAHOLD, named in TRUE nanoseconds. The DA silicon
/// SWAPS the 50 ns and 300 ns selections (DS80000882C 2.14.2) and
/// twi_sdahold_bits() undoes that, so this enum means the same thing on
/// both families.
enum class TwiSdaHold : uint8_t { off = 0, ns50 = 1, ns300 = 2, ns500 = 3 };

/// The SDAHOLD field value that really produces this hold time here.
constexpr uint8_t twi_sdahold_code(TwiSdaHold h) {
#if defined(MVIO)
    // DB: the encoding is straight (MVIO is the DB's own peripheral and
    // the cheapest way to tell the two families apart in the headers).
    return static_cast<uint8_t>(h);
#else
    // DA errata 2.14.2: use the 50 ns selection for 300 ns and vice versa.
    if (h == TwiSdaHold::ns50) return static_cast<uint8_t>(TwiSdaHold::ns300);
    if (h == TwiSdaHold::ns300) return static_cast<uint8_t>(TwiSdaHold::ns50);
    return static_cast<uint8_t>(h);
#endif
}

/// The hold time a SDAHOLD field value really produces (the inverse of
/// the line above - the swap is its own inverse).
constexpr TwiSdaHold twi_sdahold_of(uint8_t code) {
    return static_cast<TwiSdaHold>(twi_sdahold_code(static_cast<TwiSdaHold>(code & 0x03u)));
}

/// The CTRLA bits for a hold time (already in place, bits 3:2).
constexpr uint8_t twi_sdahold_bits(TwiSdaHold h) {
    return static_cast<uint8_t>(twi_sdahold_code(h) << TWI_SDAHOLD_gp);
}

/// MCTRLA.TIMEOUT: the SMBus inactive-bus supervisor. A non-zero value
/// puts the bus state machine back to Idle after that much silence.
enum class TwiTimeout : uint8_t { disabled = 0, us50 = 1, us100 = 2, us200 = 3 };

/// MCTRLB.ACKACT / SCTRLB.ACKACT: what the next command sends.
enum class TwiAck : uint8_t { ack = 0, nack = 1 };

/// MCTRLB.MCMD (table 29-2): the host's strobes. Every one of them
/// executes the acknowledge action first.
enum class TwiHostCmd : uint8_t {
    none = 0,           ///< reserved: writing it does nothing
    repeat_start = 1,   ///< ACKACT, then a repeated START
    recv_trans = 2,     ///< ACKACT, then a byte read (R) / a byte write (W)
    stop = 3,           ///< ACKACT, then a STOP
};

/// SCTRLB.SCMD (table 29-3): the client's strobes.
enum class TwiClientCmd : uint8_t {
    none = 0,           ///< no action
    complete = 2,       ///< COMPTRANS: ACKACT, then wait for any (repeated) START
    response = 3,       ///< RESPONSE: ACKACT, then the next byte
};

/// MSTATUS.BUSSTATE (29.5.6, figure 29-4).
enum class TwiBusState : uint8_t { unknown = 0, idle = 1, owner = 2, busy = 3 };

// ---- the baud arithmetic (29.3.2.2.1) ---------------------------------------

/// The three bus speed classes of this peripheral. Fast-mode Plus needs
/// CTRLA.FMPEN as well - it is what changes the PADS (x10 drive instead
/// of the slew limit, 29.3.3.1), the divider knows nothing about it.
enum class TwiSpeed : uint8_t {
    standard_100k,   ///< Sm, up to 100 kHz
    fast_400k,       ///< Fm, up to 400 kHz
    fast_plus_1m,    ///< Fm+, up to 1 MHz - requires FMPEN
};

constexpr uint32_t twi_scl_hz(TwiSpeed s) {
    switch (s) {
        case TwiSpeed::standard_100k: return 100'000u;
        case TwiSpeed::fast_400k: return 400'000u;
        default: return 1'000'000u;
    }
}

/// The rise-time BUDGET the BAUD calculation charges to the bus: the
/// I2C specification's maximum tR for the mode. The real bus is faster
/// with stiff pull-ups, which makes the real SCL run a little ABOVE the
/// value this budget predicts - measure it, do not assume it
/// (twi_scl_hz_at() takes the rise time as an argument for that reason).
constexpr uint32_t twi_rise_budget_ns(TwiSpeed s) {
    switch (s) {
        case TwiSpeed::standard_100k: return 1000u;
        case TwiSpeed::fast_400k: return 300u;
        default: return 120u;
    }
}

/// The fall-time BUDGET the calculation charges to the bus: the I2C
/// specification's maximum tOF for the mode. It matters because the SCL
/// LOW time the pins really show is the register's (BAUD + 5) clocks
/// MINUS the fall (equation 29-4) - charge nothing for it and the low
/// period lands below the mode's floor by exactly tOF, which the bench
/// measures. Fast-mode Plus drives the pads ten times harder, and its
/// budget is correspondingly smaller.
constexpr uint32_t twi_fall_budget_ns(TwiSpeed s) {
    switch (s) {
        case TwiSpeed::standard_100k: return 300u;
        case TwiSpeed::fast_400k: return 300u;
        default: return 120u;
    }
}

/// The specified minimum SCL low time of the mode (29.3.2.2.1 step 3).
constexpr uint32_t twi_low_min_ns(TwiSpeed s) {
    switch (s) {
        case TwiSpeed::standard_100k: return 4700u;
        case TwiSpeed::fast_400k: return 1300u;
        default: return 500u;
    }
}

/// Does this speed require Fast-mode Plus pads?
constexpr bool twi_needs_fm_plus(TwiSpeed s) { return s == TwiSpeed::fast_plus_1m; }

/**
 * MBAUD for a speed at a peripheral clock - the chapter's own two-step
 * calculation (29.3.2.2.1), with the bus's rise and fall times as
 * arguments because that is what the equations take:
 *   1. equation 29-3, BAUD = f_CLK/(2 f_SCL) - (5 + f_CLK tR / 2);
 *   2. equation 29-4, tLOW = (BAUD + 5)/f_CLK - tOF;
 *   3. if that tLOW is below the mode's floor, equation 29-5,
 *      BAUD = f_CLK (tLOW_mode + tOF) - 5.
 *
 * `rise_ns` and `fall_ns` describe THIS bus; 0 means "charge the mode's
 * specification maximum", which is the safe default and the only choice
 * a driver can make on its own. Declaring the real numbers of a stiff
 * bus buys speed back: they only ever appear as terms that LENGTHEN the
 * period, so a smaller pair means a faster bus - and the floor check of
 * step 3 keeps it legal. The rounding is deliberately one-sided: the
 * first term and step 3 round UP, the rise term DOWN, all three of
 * which lengthen the period.
 *
 * Empty when even BAUD = 255 cannot reach the speed at this clock.
 */
constexpr std::optional<uint8_t> twi_baud_for(uint32_t clk_hz, TwiSpeed s,
                                              uint32_t rise_ns = 0, uint32_t fall_ns = 0) {
    if (clk_hz == 0) return {};
    if (rise_ns == 0) rise_ns = twi_rise_budget_ns(s);
    if (fall_ns == 0) fall_ns = twi_fall_budget_ns(s);
    const uint32_t f_scl = twi_scl_hz(s);
    const uint64_t rise = static_cast<uint64_t>(clk_hz) * rise_ns / 2'000'000'000ull;
    const int32_t first = static_cast<int32_t>((clk_hz + 2 * f_scl - 1) / (2 * f_scl));
    int32_t baud = first - 5 - static_cast<int32_t>(rise);
    if (baud < 0) baud = 0;
    // tLOW = (BAUD + 5)/f_CLK - tOF, in nanoseconds, against the floor.
    const uint64_t low_ns = static_cast<uint64_t>(baud + 5) * 1'000'000'000ull / clk_hz;
    if (low_ns < static_cast<uint64_t>(twi_low_min_ns(s)) + fall_ns) {
        const uint64_t need = (static_cast<uint64_t>(clk_hz) *
                                   (twi_low_min_ns(s) + fall_ns) +
                               999'999'999ull) / 1'000'000'000ull;
        baud = static_cast<int32_t>(need) - 5;
        if (baud < 0) baud = 0;
    }
    if (baud > 255) return {};
    return static_cast<uint8_t>(baud);
}

/// The SCL LOW time this MBAUD produces, in peripheral clock ticks
/// (equation 29-4 with tOF = 0): the number a pulse-width meter on the
/// SCL pin should read.
constexpr uint16_t twi_low_ticks(uint8_t baud) { return static_cast<uint16_t>(baud + 5u); }

/// The SCL PERIOD this MBAUD produces, in peripheral clock ticks, on a
/// bus whose rise time is `t_rise_ns` (equation 29-2 rearranged). With
/// t_rise_ns = 0 this is the fastest the pair can possibly run: the
/// floor a frequency meter must never read below.
constexpr uint32_t twi_period_ticks(uint32_t clk_hz, uint8_t baud, uint32_t t_rise_ns) {
    return 10u + 2u * baud +
           static_cast<uint32_t>(static_cast<uint64_t>(clk_hz) * t_rise_ns / 1'000'000'000ull);
}

/// The SCL frequency this MBAUD produces at a peripheral clock, on a
/// bus whose rise time is `t_rise_ns` (equation 29-2).
constexpr uint32_t twi_scl_hz_at(uint32_t clk_hz, uint8_t baud, uint32_t t_rise_ns) {
    const uint32_t ticks = twi_period_ticks(clk_hz, baud, t_rise_ns);
    return ticks ? clk_hz / ticks : 0;
}

/// CLK_PER must be at least four times f_SCL for the bus error detector
/// (29.5.6) and the client Stop interrupt (29.5.10) to work.
constexpr bool twi_clock_ok(uint32_t clk_hz, TwiSpeed s) {
    return clk_hz >= 4u * twi_scl_hz(s);
}

// ---- the configuration ------------------------------------------------------

/// Everything one instance is configured with. The host half, the
/// client half and the dual-mode client are independent: an instance
/// may be a host, a client, or both at once (29.3.3.4 - "combined" when
/// they share the route's main pins, "dual" when the client moves to
/// the route's second pair).
struct TwiConfig {
    TwiRoute route = TwiRoute::def;

    // CTRLA - shared by the host and by a client that is NOT in Dual mode.
    TwiInputLevel input_level = TwiInputLevel::i2c;
    TwiSdaSetup sda_setup = TwiSdaSetup::cycles4;
    TwiSdaHold sda_hold = TwiSdaHold::off;
    bool fm_plus = false;
    bool debug_run = false;             ///< DBGCTRL.DBGRUN

    // MCTRLA / MBAUD.
    bool host = true;
    TwiSpeed speed = TwiSpeed::standard_100k;
    /// The bus's own rise and fall times, in nanoseconds; 0 = charge the
    /// mode's specification maximum. They enter equations 29-3 and 29-5
    /// and nothing else - see twi_baud_for().
    uint16_t rise_ns = 0;
    uint16_t fall_ns = 0;
    TwiTimeout timeout = TwiTimeout::disabled;
    bool quick_command = false;         ///< QCEN: the address packet IS the transaction
    bool host_smart = false;            ///< MCTRLA.SMEN
    bool read_interrupt = false;        ///< RIEN
    bool write_interrupt = false;       ///< WIEN

    // SCTRLA / SADDR / SADDRMASK.
    bool client = false;
    uint8_t address = 0;                ///< 7-bit, unshifted
    bool general_call = false;          ///< SADDR bit 0: also answer address 0x00
    uint8_t address_mask = 0;           ///< SADDRMASK[7:1], 7-bit
    bool second_address = false;        ///< ADDREN: the mask field is a SECOND address
    bool promiscuous = false;           ///< PMEN: answer every address
    bool client_smart = false;          ///< SCTRLA.SMEN
    bool data_interrupt = false;        ///< DIEN
    bool address_interrupt = false;     ///< APIEN
    bool stop_interrupt = false;        ///< PIEN: let a STOP raise APIF too

    // DUALCTRL - the client on the route's SECOND pin pair, with its own
    // copies of the CTRLA knobs (SDASETUP stays the shared one).
    bool dual = false;
    TwiInputLevel dual_input_level = TwiInputLevel::i2c;
    TwiSdaHold dual_sda_hold = TwiSdaHold::off;
    bool dual_fm_plus = false;
};

/// Is this configuration legal on this package?
///  - the route must exist here;
///  - Fast-mode Plus needs its pads: a 1 MHz speed without FMPEN is
///    refused rather than run on slew-limited drivers (29.3.3.1);
///  - a Dual mode client needs a bonded dual pair, and needs the client
///    enabled at all;
///  - a 7-bit address and a 7-bit mask are 7 bits;
///  - an instance that enables neither half has nothing to do.
template <uint8_t n>
constexpr bool twi_config_valid(const TwiConfig& c) {
    if (n >= twi_instance_count) return false;
    if (!twi_route_exists(n, c.route)) return false;
    if (twi_needs_fm_plus(c.speed) && !c.fm_plus) return false;
    if (c.dual && (!c.client || !twi_has_dual(n, c.route))) return false;
    if (c.address > 0x7Fu || c.address_mask > 0x7Fu) return false;
    if (!c.host && !c.client) return false;
    return true;
}

// ---- the resource -----------------------------------------------------------

template <uint8_t n>
class Twi {
    static_assert(n <= 1, "AVR DA/DB carry TWI0 and TWI1");
    static_assert(n < twi_instance_count,
                  "this package does not carry this TWI instance (TWI1 arrives "
                  "with the 32-pin step)");

public:
    Twi() = delete;

    static constexpr uint8_t index = n;

    // ---- the route table of this instance --------------------------------

    static constexpr bool has_route(TwiRoute r) { return twi_route_exists(n, r); }
    static constexpr TwiPin sda(TwiRoute r) { return twi_pin(n, r, TwiSignal::sda); }
    static constexpr TwiPin scl(TwiRoute r) { return twi_pin(n, r, TwiSignal::scl); }
    static constexpr TwiPin dual_sda(TwiRoute r) { return twi_dual_pin(n, r, TwiSignal::sda); }
    static constexpr TwiPin dual_scl(TwiRoute r) { return twi_dual_pin(n, r, TwiSignal::scl); }
    static constexpr bool has_dual(TwiRoute r) { return twi_has_dual(n, r); }

    /// The route in force since the last init()/release().
    static TwiRoute route() { return route_; }

    // ---- configuration ---------------------------------------------------

    /// Compile-time form: what this package cannot bond is refused here.
    template <TwiConfig cfg>
    static bool init(uint32_t clk_per_hz) {
        static_assert(twi_route_exists(n, cfg.route),
                      "this package does not bond this TWI route (TWI1 ALT2 needs "
                      "48 pins; TWI1 itself needs 32)");
        static_assert(twi_config_valid<n>(cfg),
                      "this TWI configuration is not legal on this package: a Dual "
                      "mode client needs a bonded dual pin pair and an enabled "
                      "client, Fast-mode Plus needs CTRLA.FMPEN, addresses are 7 "
                      "bits, and an instance with neither half enabled does nothing");
        return init(cfg, clk_per_hz);
    }

    /// Run-time form. Stops both halves, routes and prepares the pins
    /// (errata 2.15.1/2.14.1: OUT cleared before the peripheral takes
    /// them), writes the whole register set, then enables what the
    /// config asks for and declares the bus idle. False (and nothing
    /// programmed) when the config is not legal on this package or the
    /// speed cannot be reached at this clock.
    static bool init(const TwiConfig& cfg, uint32_t clk_per_hz) {
        if (!twi_config_valid<n>(cfg)) return false;
        const auto baud = twi_baud_for(clk_per_hz, cfg.speed, cfg.rise_ns, cfg.fall_ns);
        if (cfg.host && !baud) return false;
        rise_ns_ = cfg.rise_ns;
        fall_ns_ = cfg.fall_ns;
        auto& t = regs();
        t.MCTRLA = 0;                       // both halves off: the pins go back to PORT
        t.SCTRLA = 0;
        t.DUALCTRL = 0;
        setup_pins(cfg);
        t.CTRLA = ctrla_byte(cfg);
        t.DBGCTRL = cfg.debug_run ? TWI_DBGRUN_bm : 0;
        if (cfg.dual) t.DUALCTRL = dualctrl_byte(cfg);
        if (cfg.client) {
            t.SADDR = static_cast<uint8_t>((cfg.address << 1) | (cfg.general_call ? 1u : 0u));
            t.SADDRMASK = static_cast<uint8_t>((cfg.address_mask << 1) |
                                               (cfg.second_address ? 1u : 0u));
            t.SSTATUS = static_cast<uint8_t>(TWI_DIF_bm | TWI_APIF_bm | TWI_COLL_bm |
                                             TWI_BUSERR_bm);
            t.SCTRLA = sctrla_byte(cfg);
        }
        if (cfg.host) {
            t.MBAUD = *baud;
            speed_ = cfg.speed;
            t.MSTATUS = static_cast<uint8_t>(TWI_RIF_bm | TWI_WIF_bm | TWI_ARBLOST_bm |
                                             TWI_BUSERR_bm);
            t.MCTRLA = mctrla_byte(cfg);
            force_idle();                   // 29.3.2.1.1: the bus is ours to declare idle
        }
        clk_per_hz_ = clk_per_hz;
        return true;
    }

    /// Stop the instance and hand its pins back: both halves disabled,
    /// dual mode off, interrupts off, every pin this driver routed left
    /// an input with OUT = 0 (the state the erratum wants of the next
    /// owner), PORTMUX back to DEFAULT. The TWI has no pinless route, so
    /// DEFAULT - the reset code - is as far back as a release can go.
    static void release() {
        auto& t = regs();
        t.MCTRLA = 0;
        t.SCTRLA = 0;
        t.DUALCTRL = 0;
        t.CTRLA = 0;
        t.DBGCTRL = 0;
        quiet_pins(route_, true);
        write_route(TwiRoute::def);
        route_ = TwiRoute::def;
    }

    // ---- CTRLA (29.5.1) --------------------------------------------------

    /// The shared knobs. The chapter wants them written BEFORE the
    /// halves are enabled (29.3.2.1), so each of these verbs takes the
    /// enabled halves down and puts them back - which is free at
    /// configuration time and destructive in the middle of a tenure.
    static void input_level(TwiInputLevel l) {
        with_halves_down([l] {
            ctrla_bits(TWI_INPUTLVL_bm, l == TwiInputLevel::smbus);
        });
    }
    static TwiInputLevel input_level() {
        return (regs().CTRLA & TWI_INPUTLVL_bm) ? TwiInputLevel::smbus : TwiInputLevel::i2c;
    }

    static void sda_setup(TwiSdaSetup s) {
        with_halves_down([s] {
            ctrla_bits(TWI_SDASETUP_bm, s == TwiSdaSetup::cycles8);
        });
    }
    static TwiSdaSetup sda_setup() {
        return (regs().CTRLA & TWI_SDASETUP_bm) ? TwiSdaSetup::cycles8 : TwiSdaSetup::cycles4;
    }

    static void sda_hold(TwiSdaHold h) {
        with_halves_down([h] {
            regs().CTRLA = static_cast<uint8_t>((regs().CTRLA & ~TWI_SDAHOLD_gm) |
                                                twi_sdahold_bits(h));
        });
    }
    /// The hold time in force, in TRUE nanoseconds (the DA swap undone).
    static TwiSdaHold sda_hold() {
        return twi_sdahold_of(static_cast<uint8_t>((regs().CTRLA & TWI_SDAHOLD_gm) >>
                                                   TWI_SDAHOLD_gp));
    }
    /// The raw SDAHOLD field, for a caller that wants to see the swap.
    static uint8_t sda_hold_code() {
        return static_cast<uint8_t>((regs().CTRLA & TWI_SDAHOLD_gm) >> TWI_SDAHOLD_gp);
    }

    static void fm_plus(bool on) { with_halves_down([on] { ctrla_bits(TWI_FMPEN_bm, on); }); }
    static bool fm_plus() { return (regs().CTRLA & TWI_FMPEN_bm) != 0; }

    // ---- DUALCTRL (29.5.2) -----------------------------------------------

    /// Dual mode: the client moves to the route's SECOND pin pair and
    /// takes its own copies of INPUTLVL/SDAHOLD/FMPEN. False when this
    /// package does not bond that pair.
    static bool dual_mode(bool on, TwiInputLevel lvl = TwiInputLevel::i2c,
                          TwiSdaHold hold = TwiSdaHold::off, bool fmp = false) {
        if (on && !twi_has_dual(n, route_)) return false;
        if (on) quiet_pins(route_, false, true);
        regs().DUALCTRL = on ? static_cast<uint8_t>(TWI_ENABLE_bm |
                                                    (lvl == TwiInputLevel::smbus ? TWI_INPUTLVL_bm : 0) |
                                                    twi_sdahold_bits(hold) |
                                                    (fmp ? TWI_FMPEN_bm : 0))
                             : 0;
        return true;
    }
    static bool dual_mode() { return (regs().DUALCTRL & TWI_ENABLE_bm) != 0; }

    // ---- DBGCTRL (29.5.3) ------------------------------------------------

    static void debug_run(bool on) { regs().DBGCTRL = on ? TWI_DBGRUN_bm : 0; }
    static bool debug_run() { return (regs().DBGCTRL & TWI_DBGRUN_bm) != 0; }

    // ---- MCTRLA (29.5.4) -------------------------------------------------

    static void host_enable(bool on) { mctrla_bits(TWI_ENABLE_bm, on); }
    static bool host_enabled() { return (regs().MCTRLA & TWI_ENABLE_bm) != 0; }

    static void enable_read_interrupt(bool on) { mctrla_bits(TWI_RIEN_bm, on); }
    static void enable_write_interrupt(bool on) { mctrla_bits(TWI_WIEN_bm, on); }
    static uint8_t host_interrupts() {
        return static_cast<uint8_t>(regs().MCTRLA & (TWI_RIEN_bm | TWI_WIEN_bm));
    }

    /// QCEN: the address packet alone is the transaction - no data byte
    /// is exchanged and RIF or WIF is set by the direction bit
    /// (29.3.3.5). Software still has to issue the STOP.
    static void quick_command(bool on) { mctrla_bits(TWI_QCEN_bm, on); }
    static bool quick_command() { return (regs().MCTRLA & TWI_QCEN_bm) != 0; }

    /// The SMBus inactive-bus supervisor. 29.3.3.1 recommends writing
    /// MBAUD first: the time-out counter is derived from it.
    static void timeout(TwiTimeout t) {
        regs().MCTRLA = static_cast<uint8_t>((regs().MCTRLA & ~TWI_TIMEOUT_gm) |
                                             static_cast<uint8_t>(static_cast<uint8_t>(t)
                                                                  << TWI_TIMEOUT_gp));
    }
    static TwiTimeout timeout() {
        return static_cast<TwiTimeout>((regs().MCTRLA & TWI_TIMEOUT_gm) >> TWI_TIMEOUT_gp);
    }

    /// Host Smart mode: the acknowledge action goes out as soon as
    /// MDATA is READ, so a receiving handler needs no MCMD at all.
    static void host_smart(bool on) { mctrla_bits(TWI_SMEN_bm, on); }
    static bool host_smart() { return (regs().MCTRLA & TWI_SMEN_bm) != 0; }

    // ---- MCTRLB (29.5.5) -------------------------------------------------

    /// ACKACT and MCMD in ONE store - the register description says they
    /// may be written together and that the acknowledge action is taken
    /// first (table 29-2 note 1).
    ///
    /// FLUSH is deliberately absent: DB errata 2.15.2 / DA 2.14.3 say a
    /// flush can leave the host stuck in the Unknown bus state on every
    /// silicon revision. recover() is the documented work-around.
    static void host_command(TwiHostCmd cmd, TwiAck ack = TwiAck::ack) {
        regs().MCTRLB = static_cast<uint8_t>((ack == TwiAck::nack ? TWI_ACKACT_bm : 0) |
                                             static_cast<uint8_t>(cmd));
    }
    /// ACKACT alone (what Smart mode will send on the next MDATA read).
    static void ack_action(TwiAck a) {
        regs().MCTRLB = (a == TwiAck::nack) ? TWI_ACKACT_bm : 0;
    }

    /// The work-around for a wedged host: an ENABLE cycle (which is what
    /// the errata prescribe instead of FLUSH), then the bus declared
    /// idle again - an ENABLE cycle leaves BUSSTATE Unknown (29.3.2.2.2).
    static void recover() {
        const uint8_t m = regs().MCTRLA;
        regs().MCTRLA = static_cast<uint8_t>(m & ~TWI_ENABLE_bm);
        regs().MCTRLA = m;
        force_idle();
    }

    // ---- the stuck WIRE (not the peripheral) -------------------------------

    /// What unstick() returns when SDA never came back high.
    static constexpr uint8_t unstick_failed = 0xFF;
    /// The half period of unstick()'s bit-banged SCL, in microseconds
    /// (100 kHz - slow enough for any client that is worth recovering).
    static constexpr uint8_t unstick_half_us = 5;

    /**
     * The CLASSIC bus recovery, and the one thing recover() cannot do.
     *
     * recover() fixes this peripheral's own state machine. It cannot fix
     * the WIRE: a client interrupted in the middle of a byte (a reset
     * host, a glitch) goes on holding SDA low waiting for the clock
     * edges that would let it finish, and while SDA is low no host on
     * the bus can even produce a START. The remedy every I2C note gives
     * is mechanical - clock SCL up to nine times, which is one whole
     * byte plus its acknowledge, so the stuck client runs out of bits
     * and releases the line, then issue a STOP to put every device back
     * into a defined state.
     *
     * Both halves are taken down first, so the pins belong to PORT
     * again, and the bit-bang is open drain BY HAND: DIRSET over a clear
     * OUT bit pulls a line down, DIRCLR lets the pull-up take it back. A
     * line is never driven high - this is a shared bus. The pins are
     * left inputs with OUT = 0 (errata 2.15.1 / 2.14.1 hygiene) and the
     * halves are put back exactly as they were, with the bus declared
     * Idle again when the host is among them.
     *
     * Returns how many SCL pulses SDA needed (0 when it was already
     * high - the healthy case, which costs only the closing STOP) or
     * `unstick_failed` when it never came back inside `max_pulses`: a
     * line held low by something that is not counting clocks (a shorted
     * wire, a dead driver) is not a recoverable condition and the caller
     * has to be told so.
     *
     * The POLICY above this - noticing that a transaction is stuck at
     * all, and deciding to call this - belongs to the bus AO and is not
     * built (design/i2c-bus.md).
     */
    static uint8_t unstick(uint8_t max_pulses = 9) {
        const TwiPin sda_p = twi_pin(n, route_, TwiSignal::sda);
        const TwiPin scl_p = twi_pin(n, route_, TwiSignal::scl);
        if (!sda_p.bonded || !scl_p.bonded) return unstick_failed;
        if (!port_exists(sda_p.port) || !port_exists(scl_p.port)) return unstick_failed;

        const uint8_t m = regs().MCTRLA;
        const uint8_t s = regs().SCTRLA;
        regs().MCTRLA = static_cast<uint8_t>(m & ~TWI_ENABLE_bm);
        regs().SCTRLA = static_cast<uint8_t>(s & ~TWI_ENABLE_bm);

        volatile PORT_t& sda_port = port_by_letter(sda_p.port);
        volatile PORT_t& scl_port = port_by_letter(scl_p.port);
        const uint8_t sda_bm = static_cast<uint8_t>(1u << sda_p.pin);
        const uint8_t scl_bm = static_cast<uint8_t>(1u << scl_p.pin);
        sda_port.OUTCLR = sda_bm;           // the low level of every pull below
        scl_port.OUTCLR = scl_bm;
        sda_port.DIRCLR = sda_bm;           // released: the pull-up owns the line
        scl_port.DIRCLR = scl_bm;

        const uint8_t cpu = cycles_per_us(clk_per_hz_ ? clk_per_hz_ : 1'000'000u);
        uint8_t pulses = 0;
        while ((sda_port.IN & sda_bm) == 0 && pulses < max_pulses) {
            scl_port.DIRSET = scl_bm;       // SCL low
            delay_us_runtime(cpu, unstick_half_us);
            scl_port.DIRCLR = scl_bm;       // and back to the pull-up
            delay_us_runtime(cpu, unstick_half_us);
            ++pulses;
        }
        const bool freed = (sda_port.IN & sda_bm) != 0;

        // The closing STOP: SDA taken low while SCL is LOW (a data bit,
        // not a START), then SCL released, then SDA released - the SDA
        // rise against a high SCL that every device reads as Stop.
        scl_port.DIRSET = scl_bm;
        delay_us_runtime(cpu, unstick_half_us);
        sda_port.DIRSET = sda_bm;
        delay_us_runtime(cpu, unstick_half_us);
        scl_port.DIRCLR = scl_bm;
        delay_us_runtime(cpu, unstick_half_us);
        sda_port.DIRCLR = sda_bm;
        delay_us_runtime(cpu, unstick_half_us);

        regs().SCTRLA = s;
        regs().MCTRLA = m;
        if (m & TWI_ENABLE_bm) force_idle();
        return freed ? pulses : unstick_failed;
    }

    // ---- MSTATUS (29.5.6) ------------------------------------------------

    static uint8_t host_status() { return regs().MSTATUS; }
    static bool read_flag() { return (regs().MSTATUS & TWI_RIF_bm) != 0; }
    static bool write_flag() { return (regs().MSTATUS & TWI_WIF_bm) != 0; }
    static bool clock_hold() { return (regs().MSTATUS & TWI_CLKHOLD_bm) != 0; }
    /// The last acknowledge bit the CLIENT sent: true = NACK.
    static bool rx_nack() { return (regs().MSTATUS & TWI_RXACK_bm) != 0; }
    static bool arbitration_lost() { return (regs().MSTATUS & TWI_ARBLOST_bm) != 0; }
    static bool bus_error() { return (regs().MSTATUS & TWI_BUSERR_bm) != 0; }
    static TwiBusState bus_state() {
        return static_cast<TwiBusState>(regs().MSTATUS & TWI_BUSSTATE_gm);
    }

    /// Write-one-to-clear, as a PLAIN store of exactly the named bits -
    /// an RMW would write back every flag it read (and BUSSTATE with
    /// them). Only 0x1 in the BUSSTATE field has an effect, so a clear
    /// never moves the bus state by accident.
    static void clear_host_flags(uint8_t mask) { regs().MSTATUS = mask; }
    /// Force the bus state machine to Idle (the only value BUSSTATE
    /// accepts from software).
    static void force_idle() { regs().MSTATUS = TWI_BUSSTATE_IDLE_gc; }

    // ---- MBAUD / MADDR / MDATA (29.5.7 - 29.5.9) -------------------------

    /// MBAUD "must be written while the host is disabled" (29.5.7), so
    /// this verb takes the host down around the write and declares the
    /// bus idle again afterwards - an ENABLE cycle leaves BUSSTATE
    /// Unknown. Between tenures only: it destroys one in flight.
    static void set_baud(uint8_t v) {
        const uint8_t m = regs().MCTRLA;
        if (m & TWI_ENABLE_bm) {
            regs().MCTRLA = static_cast<uint8_t>(m & ~TWI_ENABLE_bm);
            regs().MBAUD = v;
            regs().MCTRLA = m;
            force_idle();
        } else {
            regs().MBAUD = v;
        }
    }
    static uint8_t baud() { return regs().MBAUD; }

    /// Writing MADDR issues the START (or the repeated START) and shifts
    /// the address packet out; bit 0 is the direction.
    static void address_write(uint8_t addr7) {
        regs().MADDR = static_cast<uint8_t>(addr7 << 1);
    }
    static void address_read(uint8_t addr7) {
        regs().MADDR = static_cast<uint8_t>((addr7 << 1) | 1u);
    }
    /// The raw register (reading it disturbs nothing, 29.5.8).
    static void maddr(uint8_t v) { regs().MADDR = v; }
    static uint8_t maddr() { return regs().MADDR; }

    /// MDATA is the shift register itself: a WRITE commands a byte
    /// transmit, a READ takes the received byte (and, in Smart mode,
    /// sends the acknowledge action).
    static void host_write(uint8_t v) { regs().MDATA = v; }
    static uint8_t host_read() { return regs().MDATA; }

    // ---- SCTRLA (29.5.10) ------------------------------------------------

    static void client_enable(bool on) { sctrla_bits(TWI_ENABLE_bm, on); }
    static bool client_enabled() { return (regs().SCTRLA & TWI_ENABLE_bm) != 0; }

    static void enable_data_interrupt(bool on) { sctrla_bits(TWI_DIEN_bm, on); }
    static void enable_address_interrupt(bool on) { sctrla_bits(TWI_APIEN_bm, on); }
    /// PIEN: without it a STOP does not raise APIF at all. Needs
    /// CLK_PER >= 4 x f_SCL (29.5.10).
    static void enable_stop_interrupt(bool on) { sctrla_bits(TWI_PIEN_bm, on); }
    static uint8_t client_interrupts() {
        return static_cast<uint8_t>(regs().SCTRLA & (TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm));
    }

    /// PMEN: answer EVERY address on the bus (29.3.2.3).
    static void promiscuous(bool on) { sctrla_bits(TWI_PMEN_bm, on); }
    static bool promiscuous() { return (regs().SCTRLA & TWI_PMEN_bm) != 0; }

    /// Client Smart mode: touching SDATA clears DIF and continues the
    /// operation, so a handler needs no SCMD.
    static void client_smart(bool on) { sctrla_bits(TWI_SMEN_bm, on); }
    static bool client_smart() { return (regs().SCTRLA & TWI_SMEN_bm) != 0; }

    // ---- SCTRLB (29.5.11) ------------------------------------------------

    /// ACKACT and SCMD in one store (table 29-3 note 1: the acknowledge
    /// action is updated before the command triggers).
    static void client_command(TwiClientCmd cmd, TwiAck ack = TwiAck::ack) {
        regs().SCTRLB = static_cast<uint8_t>((ack == TwiAck::nack ? TWI_ACKACT_bm : 0) |
                                             static_cast<uint8_t>(cmd));
    }
    static void client_ack_action(TwiAck a) {
        regs().SCTRLB = (a == TwiAck::nack) ? TWI_ACKACT_bm : 0;
    }

    // ---- SSTATUS (29.5.12) -----------------------------------------------

    static uint8_t client_status() { return regs().SSTATUS; }
    static bool data_flag() { return (regs().SSTATUS & TWI_DIF_bm) != 0; }
    static bool address_flag() { return (regs().SSTATUS & TWI_APIF_bm) != 0; }
    static bool client_clock_hold() { return (regs().SSTATUS & TWI_CLKHOLD_bm) != 0; }
    /// The last acknowledge bit the HOST sent: true = NACK.
    static bool client_rx_nack() { return (regs().SSTATUS & TWI_RXACK_bm) != 0; }
    /// COLL: this client could not put a high bit (or a NACK) on SDA.
    static bool collision() { return (regs().SSTATUS & TWI_COLL_bm) != 0; }
    static bool client_bus_error() { return (regs().SSTATUS & TWI_BUSERR_bm) != 0; }
    /// DIR: true while the host is READING from this client.
    static bool host_reading() { return (regs().SSTATUS & TWI_DIR_bm) != 0; }
    /// AP: what raised APIF - an address match (true) or a STOP (false).
    static bool address_match() { return (regs().SSTATUS & TWI_AP_bm) != 0; }

    /// The write-one-to-clear client flags (DIF, APIF, COLL, BUSERR), as
    /// a plain store of exactly the named bits.
    static void clear_client_flags(uint8_t mask) { regs().SSTATUS = mask; }

    // ---- SADDR / SDATA / SADDRMASK (29.5.13 - 29.5.15) -------------------

    /// The client's own address, plus the General Call bit (SADDR[0]).
    static void client_address(uint8_t addr7, bool general_call = false) {
        regs().SADDR = static_cast<uint8_t>((addr7 << 1) | (general_call ? 1u : 0u));
    }
    static uint8_t client_address() { return static_cast<uint8_t>(regs().SADDR >> 1); }
    static bool general_call() { return (regs().SADDR & 0x01u) != 0; }

    /// SADDRMASK with ADDREN = 0: the set bits are the address bits the
    /// match logic IGNORES - an address RANGE.
    static void address_mask(uint8_t mask7) {
        regs().SADDRMASK = static_cast<uint8_t>(mask7 << 1);
    }
    /// SADDRMASK with ADDREN = 1: a SECOND exact address.
    static void second_address(uint8_t addr7) {
        regs().SADDRMASK = static_cast<uint8_t>((addr7 << 1) | 1u);
    }
    static uint8_t addrmask_field() { return static_cast<uint8_t>(regs().SADDRMASK >> 1); }
    static bool second_address_enabled() { return (regs().SADDRMASK & 0x01u) != 0; }

    static void client_write(uint8_t v) { regs().SDATA = v; }
    static uint8_t client_read() { return regs().SDATA; }

    // ---- ISR bodies (two vectors: TWIn_TWIM_vect, TWIn_TWIS_vect) --------

    /// What a HOST interrupt found. Nothing is cleared here: RIF/WIF
    /// clear when the handler writes MADDR, MDATA or MCMD (29.5.6), and
    /// a body that cleared them would eat the state its caller reads.
    struct HostIsr {
        uint8_t status;
        constexpr bool read_done() const { return (status & TWI_RIF_bm) != 0; }
        constexpr bool write_done() const { return (status & TWI_WIF_bm) != 0; }
        constexpr bool nack() const { return (status & TWI_RXACK_bm) != 0; }
        constexpr bool arbitration_lost() const { return (status & TWI_ARBLOST_bm) != 0; }
        constexpr bool bus_error() const { return (status & TWI_BUSERR_bm) != 0; }
        constexpr bool clock_hold() const { return (status & TWI_CLKHOLD_bm) != 0; }
        constexpr TwiBusState state() const {
            return static_cast<TwiBusState>(status & TWI_BUSSTATE_gm);
        }
    };
    [[gnu::always_inline]] static HostIsr take_host() { return {regs().MSTATUS}; }

    /// What a CLIENT interrupt found. Same discipline: DIF and APIF
    /// clear when the handler writes SCMD or touches SDATA (29.5.12).
    struct ClientIsr {
        uint8_t status;
        constexpr bool data() const { return (status & TWI_DIF_bm) != 0; }
        constexpr bool address_or_stop() const { return (status & TWI_APIF_bm) != 0; }
        constexpr bool is_address() const { return (status & TWI_AP_bm) != 0; }
        constexpr bool is_stop() const {
            return (status & TWI_APIF_bm) != 0 && (status & TWI_AP_bm) == 0;
        }
        constexpr bool host_reading() const { return (status & TWI_DIR_bm) != 0; }
        constexpr bool nack() const { return (status & TWI_RXACK_bm) != 0; }
        constexpr bool collision() const { return (status & TWI_COLL_bm) != 0; }
        constexpr bool bus_error() const { return (status & TWI_BUSERR_bm) != 0; }
        constexpr bool clock_hold() const { return (status & TWI_CLKHOLD_bm) != 0; }
    };
    [[gnu::always_inline]] static ClientIsr take_client() { return {regs().SSTATUS}; }

    // ---- registers, routing, clock ---------------------------------------

    static constexpr TWI_t& regs() {
#if defined(TWI1)
        if constexpr (n == 0) return TWI0;
        else return TWI1;
#else
        return TWI0;
#endif
    }

    /// PORTMUX position of this instance's two route bits.
    static constexpr uint8_t route_gp() {
#if defined(TWI1)
        if constexpr (n == 0) return PORTMUX_TWI0_gp;
        else return PORTMUX_TWI1_gp;
#else
        return PORTMUX_TWI0_gp;
#endif
    }

    static void write_route(TwiRoute r) {
        constexpr uint8_t gp = route_gp();
        constexpr uint8_t gm = static_cast<uint8_t>(0x03u << gp);
        PORTMUX.TWIROUTEA = static_cast<uint8_t>(
            (PORTMUX.TWIROUTEA & ~gm) | static_cast<uint8_t>(static_cast<uint8_t>(r) << gp));
    }

    /// The route bits as they read back.
    static TwiRoute routed() {
        constexpr uint8_t gp = route_gp();
        constexpr uint8_t gm = static_cast<uint8_t>(0x03u << gp);
        return static_cast<TwiRoute>((PORTMUX.TWIROUTEA & gm) >> gp);
    }

    /// The peripheral clock last seen by init()/rebase(), and whether it
    /// is at least four times a speed's SCL - the condition the bus
    /// error detector (29.5.6) and the client Stop interrupt (29.5.10)
    /// need. Not enforced: a slower clock is legal, it only blinds them.
    static uint32_t clk_per_hz() { return clk_per_hz_; }
    /// Record the peripheral clock without reprogramming anything (the
    /// baud arithmetic reads it). init() does it itself; the half that
    /// joins an already-running instance uses this.
    static void note_clock(uint32_t hz) { clk_per_hz_ = hz; }
    static bool clock_ok(TwiSpeed s) { return twi_clock_ok(clk_per_hz_, s); }
    /// The speed the last set_speed()/init() programmed.
    static TwiSpeed speed() { return speed_; }

    /// Reprogram MBAUD (and FMPEN with it) for a speed at the peripheral
    /// clock last seen. False when the speed cannot be reached, or needs
    /// pads this call may not change (FMPEN moves with the speed here:
    /// it IS the pad half of Fm+).
    /// The bus's rise and fall times (0 = the mode's specification
    /// maximum), which equations 29-3 and 29-5 charge to every MBAUD
    /// this instance computes from now on.
    static void bus_timing(uint16_t rise_ns, uint16_t fall_ns) {
        rise_ns_ = rise_ns;
        fall_ns_ = fall_ns;
    }
    static uint16_t rise_ns() { return rise_ns_; }
    static uint16_t fall_ns() { return fall_ns_; }

    static bool set_speed(TwiSpeed s) {
        const auto b = twi_baud_for(clk_per_hz_, s, rise_ns_, fall_ns_);
        if (!b) return false;
        const bool want_fmp = twi_needs_fm_plus(s);
        if (want_fmp != fm_plus()) fm_plus(want_fmp);
        set_baud(*b);
        speed_ = s;
        return true;
    }

    /// The SCL this instance really produces, given a bus rise time (0 =
    /// the fastest the divider can go).
    static uint32_t actual_scl_hz(uint32_t t_rise_ns = 0) {
        return twi_scl_hz_at(clk_per_hz_, regs().MBAUD, t_rise_ns);
    }

    /// A clock change: MBAUD is derived from CLK_PER, so it is recomputed
    /// for the speed in force. False when that speed no longer fits.
    static bool rebase(uint32_t hz) {
        clk_per_hz_ = hz;
        return set_speed(speed_);
    }

private:
    static constexpr uint8_t ctrla_byte(const TwiConfig& c) {
        return static_cast<uint8_t>(
            (c.input_level == TwiInputLevel::smbus ? TWI_INPUTLVL_bm : 0) |
            (c.sda_setup == TwiSdaSetup::cycles8 ? TWI_SDASETUP_bm : 0) |
            twi_sdahold_bits(c.sda_hold) |
            (c.fm_plus ? TWI_FMPEN_bm : 0));
    }

    static constexpr uint8_t dualctrl_byte(const TwiConfig& c) {
        return static_cast<uint8_t>(
            TWI_ENABLE_bm |
            (c.dual_input_level == TwiInputLevel::smbus ? TWI_INPUTLVL_bm : 0) |
            twi_sdahold_bits(c.dual_sda_hold) |
            (c.dual_fm_plus ? TWI_FMPEN_bm : 0));
    }

    static constexpr uint8_t mctrla_byte(const TwiConfig& c) {
        return static_cast<uint8_t>(
            TWI_ENABLE_bm |
            (c.read_interrupt ? TWI_RIEN_bm : 0) |
            (c.write_interrupt ? TWI_WIEN_bm : 0) |
            (c.quick_command ? TWI_QCEN_bm : 0) |
            static_cast<uint8_t>(static_cast<uint8_t>(c.timeout) << TWI_TIMEOUT_gp) |
            (c.host_smart ? TWI_SMEN_bm : 0));
    }

    static constexpr uint8_t sctrla_byte(const TwiConfig& c) {
        return static_cast<uint8_t>(
            TWI_ENABLE_bm |
            (c.data_interrupt ? TWI_DIEN_bm : 0) |
            (c.address_interrupt ? TWI_APIEN_bm : 0) |
            (c.stop_interrupt ? TWI_PIEN_bm : 0) |
            (c.promiscuous ? TWI_PMEN_bm : 0) |
            (c.client_smart ? TWI_SMEN_bm : 0));
    }

    static void ctrla_bits(uint8_t bit, bool on) {
        if (on) regs().CTRLA |= bit; else regs().CTRLA &= static_cast<uint8_t>(~bit);
    }
    static void mctrla_bits(uint8_t bit, bool on) {
        if (on) regs().MCTRLA |= bit; else regs().MCTRLA &= static_cast<uint8_t>(~bit);
    }
    static void sctrla_bits(uint8_t bit, bool on) {
        if (on) regs().SCTRLA |= bit; else regs().SCTRLA &= static_cast<uint8_t>(~bit);
    }

    /// The CTRLA knobs must be written with the halves down (29.3.2.1).
    template <typename F>
    static void with_halves_down(F body) {
        const uint8_t m = regs().MCTRLA;
        const uint8_t s = regs().SCTRLA;
        regs().MCTRLA = static_cast<uint8_t>(m & ~TWI_ENABLE_bm);
        regs().SCTRLA = static_cast<uint8_t>(s & ~TWI_ENABLE_bm);
        body();
        regs().SCTRLA = s;
        regs().MCTRLA = m;
        if (m & TWI_ENABLE_bm) force_idle();
    }

    /// Route, then prepare the pins the peripheral is about to take.
    ///
    /// ERRATA DB 2.15.1 / DA 2.14.1: with the TWI enabled the pin
    /// override covers the driver but NOT the value, so a PORTx.OUT bit
    /// left at '1' on SDA or SCL pins that line permanently high. Both
    /// pins therefore go back to being inputs with OUT = 0 before the
    /// peripheral takes them - unconditionally, on every revision: two
    /// stores, and the alternative is a bus that never starts.
    static void setup_pins(const TwiConfig& c) {
        quiet_pins(route_, true);            // the route we are leaving
        write_route(c.route);
        route_ = c.route;
        quiet_pins(c.route, false);
        if (c.dual) quiet_pins(c.route, false, true);
    }

    /// SDA and SCL of a route (or of its dual pair) back to inputs with
    /// OUT = 0. `both` also covers the dual pair, for the teardown that
    /// does not know what was enabled.
    static void quiet_pins(TwiRoute r, bool both, bool dual_only = false) {
        if (!twi_route_exists(n, r)) return;
        if (!dual_only) {
            quiet_pin(twi_pin(n, r, TwiSignal::sda));
            quiet_pin(twi_pin(n, r, TwiSignal::scl));
        }
        if (both || dual_only) {
            quiet_pin(twi_dual_pin(n, r, TwiSignal::sda));
            quiet_pin(twi_dual_pin(n, r, TwiSignal::scl));
        }
    }

    static void quiet_pin(TwiPin p) {
        if (!p.bonded || !port_exists(p.port)) return;
        volatile PORT_t& port = port_by_letter(p.port);
        port.OUTCLR = static_cast<uint8_t>(1u << p.pin);
        port.DIRCLR = static_cast<uint8_t>(1u << p.pin);
    }

    static inline TwiRoute route_ = TwiRoute::def;
    static inline TwiSpeed speed_ = TwiSpeed::standard_100k;
    static inline uint32_t clk_per_hz_ = 0;
    static inline uint16_t rise_ns_ = 0;   ///< 0 = the mode's specification maximum
    static inline uint16_t fall_ns_ = 0;
};

// ---- tasks ------------------------------------------------------------------

/*
 * TwiHost<n, route>
 *
 * The transfer ENGINE: the target-side half of the I2C stack, driven by
 * util/i2c_bus.hpp (the BusMaster arbiter, which owns arbitration and
 * replies). This engine owns the wire: START/repeated START/STOP, the
 * address phase, the ACK policy and the byte pump under the TWI host
 * interrupt (one interrupt per byte, the same honest price the SPI
 * engine pays).
 *
 * Transaction descriptor (Request) - ONE bus tenure, from START to STOP,
 * in the three shapes I2C devices actually use:
 *
 *   write             tx[tx_len], rx_len == 0        S addr+W data... P
 *   read              tx_len == 0, rx[rx_len]        S addr+R data... P
 *   write-then-read   both                           S addr+W tx... Sr addr+R rx... P
 *   probe             both empty                     S addr+W P     -> ACK/NACK
 *
 * The write-then-read shape is the register-access idiom (send the
 * register index, repeated START, read the value) and it MUST be one
 * request: a repeated START is what keeps another host from slipping in
 * between - the SPI rule "the request is the complete script of one bus
 * tenure" holds verbatim. The probe is what an address scanner sends:
 * the reply's status says whether anybody ACKed.
 *
 * Status codes (util/i2c_bus.hpp): i2c_ok, i2c_nack_addr, i2c_nack_data,
 * i2c_arb_lost, i2c_bus_error. On any NACK the engine still issues the
 * STOP so the bus is released; on arbitration lost the bus belongs to
 * the other host and no STOP is sent; on bus error the peripheral is
 * forced back to the idle bus state.
 *
 * Buffer ownership travels with the request (the client hands the spans
 * off until its I2cDone comes back). Bus speed travels per request, as
 * on SPI: a shared bus can carry a 100 kHz sensor and a 400 kHz DAC.
 * MBAUD may only be written with the host disabled (29.5.7), so a speed
 * CHANGE costs an ENABLE cycle and a force-idle - paid at start(), only
 * when the speed actually moves, and never per byte.
 *
 * NOT covered (noted, not built): a stuck-bus WATCHDOG - the mechanical
 * remedy is Twi<n>::unstick() (and this task's `unstick()` below), but
 * noticing that a transaction is stuck and deciding to call it is a
 * policy for the bus AO, which has no per-request timeout yet. Also not
 * covered: 10-bit addressing.
 *
 * ISR wiring (app glue, as usual):
 *   ISR(TWI0_TWIM_vect) {
 *       if (TwiHw::isr()) { brio::post<I2c>(brio::TransferDone{TwiHw::status()}); }
 *   }
 */
template <uint8_t n, TwiRoute route = TwiRoute::def>
class TwiHost {
    using T = Twi<n>;
    static_assert(twi_route_exists(n, route),
                  "this package does not bond this TWI route (TWI1 ALT2 needs 48 "
                  "pins; TWI1 itself needs 32)");

public:
    TwiHost() = delete;

    using Resource = T;
    static constexpr TwiRoute pin_route = route;
    static constexpr bool available = twi_route_exists(n, route);

    struct Request {
        uint8_t addr;          ///< 7-bit client address (unshifted)
        const uint8_t* tx;     ///< bytes written after START (may be null if tx_len == 0)
        uint8_t tx_len;
        uint8_t* rx;           ///< bytes read after the (repeated) START+R
        uint8_t rx_len;
        ReplyTo<I2cDone> reply;
        TwiSpeed speed = TwiSpeed::standard_100k;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    /// What the whole bus is configured with, beyond the per-request
    /// speed. The SMBus knobs live here because they are properties of
    /// the WIRE, not of one transaction.
    struct Options {
        TwiSpeed speed = TwiSpeed::standard_100k;   ///< the speed init() programs
        /// The rise and fall times of THIS bus in nanoseconds; 0 charges
        /// the mode's specification maximum, which is what a driver that
        /// knows nothing about the wiring must assume. A stiff bus that
        /// declares its measured numbers gets a faster SCL and still
        /// meets the mode's tLOW floor - see twi_baud_for().
        uint16_t rise_ns = 0;
        uint16_t fall_ns = 0;
        TwiTimeout timeout = TwiTimeout::disabled;
        TwiSdaHold sda_hold = TwiSdaHold::off;
        TwiSdaSetup sda_setup = TwiSdaSetup::cycles4;
        TwiInputLevel input_level = TwiInputLevel::i2c;
        bool smart = false;         ///< MCTRLA.SMEN: the ACK rides the MDATA read
        bool debug_run = false;
    };

    /**
     * Host, interrupt driven. Call after clock init, before sei(). The
     * pull-ups are external (the internal ones are far too weak for I2C
     * edges); the pins are left to the peripheral, which drives them
     * open-drain by itself - init only routes them and makes sure their
     * PORT.OUT bits are clear (errata 2.15.1 / 2.14.1).
     *
     * False when the package cannot bond the route or the speed cannot
     * be reached at this clock.
     */
    template <typename Clock>
    static bool init(Clock clock, Options o = {}) {
        static_assert(clock_follows<Clock, TwiHost>(),
                      "this TwiHost is initialized with a DynamicClock that does not "
                      "list it among its Users: MBAUD would go stale on a clock change");
        if constexpr (!available) {
            (void)clock;
            (void)o;
            return false;
        } else {
            smart_ = o.smart;
            if (T::client_enabled()) {
                // COMBINED mode: a client of this instance is already
                // running. Only the host half and the shared CTRLA knobs
                // are written - the client keeps its address and its
                // pins. The knobs ARE one register for both halves, so
                // whichever init runs last decides them; that is the
                // silicon, not a policy of this driver.
                T::note_clock(clock_hz(clock));
                T::bus_timing(o.rise_ns, o.fall_ns);
                T::host_enable(false);
                T::input_level(o.input_level);
                T::sda_setup(o.sda_setup);
                T::sda_hold(o.sda_hold);
                T::debug_run(o.debug_run);
                T::enable_read_interrupt(true);
                T::enable_write_interrupt(true);
                T::timeout(o.timeout);
                T::host_smart(o.smart);
                T::quick_command(false);
                T::host_enable(true);
                if (!T::set_speed(o.speed)) return false;
                T::force_idle();
                return true;
            }
            return T::init({.route = route,
                            .input_level = o.input_level,
                            .sda_setup = o.sda_setup,
                            .sda_hold = o.sda_hold,
                            .fm_plus = twi_needs_fm_plus(o.speed),
                            .debug_run = o.debug_run,
                            .host = true,
                            .speed = o.speed,
                            .rise_ns = o.rise_ns,
                            .fall_ns = o.fall_ns,
                            .timeout = o.timeout,
                            .host_smart = o.smart,
                            .read_interrupt = true,
                            .write_interrupt = true},
                           clock_hz(clock));
        }
    }

    /// The peripheral clock changed (DynamicClock fan-out): MBAUD is
    /// derived from CLK_PER, so it is recomputed for the speed in force.
    /// The bus must be IDLE - the rewrite costs an ENABLE cycle.
    static void rebase(uint32_t hz) { (void)T::rebase(hz); }

    /// The SCL the host really produces at the speed in force, on a bus
    /// whose rise time is `t_rise_ns` (0 = the divider's own limit).
    static uint32_t actual_scl_hz(uint32_t t_rise_ns = 0) { return T::actual_scl_hz(t_rise_ns); }
    static uint8_t baud() { return T::baud(); }
    static TwiSpeed speed() { return T::speed(); }
    /// CLK_PER >= 4 x f_SCL, the condition the bus error detector needs.
    static bool clock_ok() { return T::clock_ok(T::speed()); }

    /// QCEN: every request becomes an address-only frame (29.3.3.5) and
    /// its data spans are ignored - the shape a quick command has.
    static void quick_command(bool on) { T::quick_command(on); }
    static bool quick_command() { return T::quick_command(); }

    /// Begin a transaction (called by the bus AO from main context).
    /// Always asynchronous: returns false and a TransferDone{status()}
    /// follows from the ISR glue - even the empty probe ends on the wire
    /// (its address phase IS the transaction).
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        status_ = i2c_ok;
        quick_ = T::quick_command();
        if (r.speed != T::speed()) {
            (void)T::set_speed(r.speed);     // ENABLE cycle + force idle: between tenures
        }
        if (!quick_ && r.tx_len == 0 && r.rx_len > 0) {
            phase_ = Phase::reading;
            T::address_read(r.addr);
        } else {
            phase_ = Phase::writing;
            T::address_write(r.addr);
        }
        return false;
    }

    /// Outcome of the last transaction (valid once isr() returned true).
    static uint8_t status() { return status_; }

    /**
     * @brief TWI host interrupt body - call from ISR(TWIn_TWIM_vect).
     * @return true when the transaction just completed (STOP issued or
     * bus lost): the edge on which the glue posts TransferDone.
     */
    [[gnu::always_inline]] static bool isr() {
        const auto st = T::take_host();

        if (st.arbitration_lost()) {        // another host won: not our bus
            T::clear_host_flags(TWI_ARBLOST_bm | TWI_WIF_bm);
            return finish(i2c_arb_lost);
        }
        if (st.bus_error()) {               // protocol violation: force idle
            T::clear_host_flags(TWI_BUSERR_bm | TWI_WIF_bm);
            T::force_idle();
            return finish(i2c_bus_error);
        }
        if (st.write_done()) {              // address or data byte went out
            if (st.nack()) {                // NACK: release the bus, report
                T::host_command(TwiHostCmd::stop);
                return finish((phase_ == Phase::writing && pos_ == 0)
                                  ? i2c_nack_addr : i2c_nack_data);
            }
            if (!quick_ && phase_ == Phase::writing && pos_ < req_.tx_len) {
                T::host_write(req_.tx[pos_++]);
                return false;
            }
            if (!quick_ && req_.rx_len > 0) {   // repeated START, direction read
                phase_ = Phase::reading;
                pos_ = 0;
                T::address_read(req_.addr);
                return false;
            }
            T::host_command(TwiHostCmd::stop);  // write / probe / quick command complete
            return finish(i2c_ok);
        }
        if (st.read_done()) {               // a data byte came in
            if (quick_) {                   // a quick command in the read direction
                T::host_command(TwiHostCmd::stop);
                return finish(i2c_ok);
            }
            const bool last = static_cast<uint8_t>(pos_ + 1) >= req_.rx_len;
            if (smart_) {
                // Smart mode: the acknowledge action goes out with the
                // MDATA read, so ACKACT is armed FIRST and no receive
                // command follows - only the STOP after the last byte.
                T::ack_action(last ? TwiAck::nack : TwiAck::ack);
                req_.rx[pos_++] = T::host_read();
                if (!last) return false;
                T::host_command(TwiHostCmd::stop, TwiAck::nack);
                return finish(i2c_ok);
            }
            req_.rx[pos_++] = T::host_read();
            if (!last) {
                T::host_command(TwiHostCmd::recv_trans, TwiAck::ack);
                return false;
            }
            T::host_command(TwiHostCmd::stop, TwiAck::nack);
            return finish(i2c_ok);
        }
        return false;                       // spurious: nothing to do
    }

    /// The errata's work-around for a wedged host (FLUSH is not usable:
    /// DB 2.15.2 / DA 2.14.3): an ENABLE cycle, then the bus idle again.
    static void recover() { T::recover(); }
    /// The other half of recovery: the WIRE, not the peripheral. See
    /// Twi<n>::unstick() - up to `max_pulses` bit-banged SCL pulses
    /// until a stuck client releases SDA, then a STOP. Returns the pulse
    /// count, or Twi<n>::unstick_failed.
    static uint8_t unstick(uint8_t max_pulses = 9) { return T::unstick(max_pulses); }
    static TwiBusState bus_state() { return T::bus_state(); }

    /// Hand the route's pins back (the resource's teardown).
    static void release() { T::release(); }

private:
    enum class Phase : uint8_t { writing, reading };

    static bool finish(uint8_t code) {
        status_ = code;
        return true;
    }

    static inline Request req_{};
    static inline uint8_t pos_ = 0;
    static inline Phase phase_ = Phase::writing;
    static inline uint8_t status_ = i2c_ok;
    static inline bool smart_ = false;
    static inline bool quick_ = false;
};

/*
 * TwiClient<n, route, on_dual_pins>
 *
 * The other end of the wire (29.3.2.3): the host sets the pace, this
 * side answers. The protocol is four cases and this task is their verbs:
 *
 *   S1  address matched, DIR = 0 (host writes) -> receive()
 *   S2  address matched, DIR = 1 (host reads)  -> transmit(byte)
 *   S3  STOP received (APIF with AP = 0, needs PIEN)
 *   S4  collision: this client could not put a high bit on SDA
 *
 * The address-match space is the whole of 29.3.2.3.1: the exact address,
 * the General Call address (SADDR bit 0), an address RANGE (SADDRMASK
 * with ADDREN = 0 masks the bits to ignore), a SECOND exact address
 * (ADDREN = 1), or every address on the bus (PMEN).
 *
 * `on_dual_pins` puts the client on the route's SECOND pin pair through
 * DUALCTRL, with its own INPUTLVL/SDAHOLD/FMPEN - the host then keeps
 * the main pair for itself. It is refused at compile time on a package
 * that does not bond that pair.
 *
 * Combined mode (a host and a client on the SAME instance and the SAME
 * pins) is legal and needs no flag: initialize both tasks. The CTRLA
 * knobs are then ONE register shared by the two halves and the later
 * init wins - which is what the silicon does, not a policy of this
 * driver.
 *
 * ISR wiring (app glue, as usual):
 *   ISR(TWI0_TWIS_vect) { const auto s = Client::isr(); ... }
 */
template <uint8_t n, TwiRoute route = TwiRoute::def, bool on_dual_pins = false>
class TwiClient {
    using T = Twi<n>;
    static_assert(twi_route_exists(n, route),
                  "this package does not bond this TWI route (TWI1 ALT2 needs 48 "
                  "pins; TWI1 itself needs 32)");
    static_assert(!on_dual_pins || twi_has_dual(n, route),
                  "this package does not bond the DUAL pin pair of this TWI route: "
                  "TWI0 ALT1/ALT2 need 48 pins for PC6/PC7, TWI1 DEFAULT needs 48 "
                  "for PB2/PB3 and TWI1 ALT1/ALT2 need 64 for PB6/PB7");

public:
    TwiClient() = delete;

    using Resource = T;
    static constexpr TwiRoute pin_route = route;
    static constexpr bool dual = on_dual_pins;
    static constexpr bool available =
        twi_route_exists(n, route) && (!on_dual_pins || twi_has_dual(n, route));

    /// Where this client's pins are - the route's main pair, or its dual
    /// pair in Dual mode.
    static constexpr TwiPin sda_pin() {
        return on_dual_pins ? twi_dual_pin(n, route, TwiSignal::sda)
                            : twi_pin(n, route, TwiSignal::sda);
    }
    static constexpr TwiPin scl_pin() {
        return on_dual_pins ? twi_dual_pin(n, route, TwiSignal::scl)
                            : twi_pin(n, route, TwiSignal::scl);
    }

    struct Options {
        uint8_t address = 0;                ///< 7-bit, unshifted
        bool general_call = false;          ///< also answer address 0x00
        uint8_t address_mask = 0;           ///< bits to IGNORE, or a second address
        bool second_address = false;        ///< ADDREN: address_mask IS a second address
        bool promiscuous = false;           ///< PMEN: answer everything
        bool smart = false;                 ///< SCTRLA.SMEN
        bool stop_interrupt = true;         ///< PIEN: let a STOP raise APIF
        bool data_interrupt = false;        ///< DIEN
        bool address_interrupt = false;     ///< APIEN
        TwiSdaSetup sda_setup = TwiSdaSetup::cycles4;
        TwiSdaHold sda_hold = TwiSdaHold::off;
        TwiInputLevel input_level = TwiInputLevel::i2c;
        bool fm_plus = false;               ///< the pads, when the host runs Fm+
    };

    /// Configure the instance as a client. In Dual mode the knobs go to
    /// DUALCTRL (its own copies); otherwise they go to the CTRLA the
    /// host shares. False when the package cannot bond the pins.
    template <typename Clock>
    static bool init(Clock clock, Options o) {
        static_assert(clock_follows<Clock, TwiClient>(),
                      "this TwiClient is initialized with a DynamicClock that does "
                      "not list it among its Users: the CLK_PER >= 4 x f_SCL "
                      "readback would go stale on a clock change");
        if constexpr (!available) {
            (void)clock;
            (void)o;
            return false;
        } else {
            clk_per_hz_ = clock_hz(clock);
            smart_ = o.smart;
            const bool host_up = T::host_enabled();
            if (host_up) {
                // COMBINED mode: the host is already running. Only the
                // client half and the shared knobs are (re)written; the
                // host keeps its baud, its route and its bus state.
                T::client_enable(false);
                T::sda_setup(o.sda_setup);
                if constexpr (on_dual_pins) {
                    if (!T::dual_mode(true, o.input_level, o.sda_hold, o.fm_plus)) return false;
                } else {
                    T::input_level(o.input_level);
                    T::sda_hold(o.sda_hold);
                    if (o.fm_plus) T::fm_plus(true);
                }
                apply_address(o);
                apply_client_bits(o);
                T::clear_client_flags(TWI_DIF_bm | TWI_APIF_bm | TWI_COLL_bm | TWI_BUSERR_bm);
                T::client_enable(true);
                return true;
            }
            return T::init({.route = route,
                            .input_level = on_dual_pins ? TwiInputLevel::i2c : o.input_level,
                            .sda_setup = o.sda_setup,
                            .sda_hold = on_dual_pins ? TwiSdaHold::off : o.sda_hold,
                            .fm_plus = on_dual_pins ? false : o.fm_plus,
                            .host = false,
                            .client = true,
                            .address = o.address,
                            .general_call = o.general_call,
                            .address_mask = o.address_mask,
                            .second_address = o.second_address,
                            .promiscuous = o.promiscuous,
                            .client_smart = o.smart,
                            .data_interrupt = o.data_interrupt,
                            .address_interrupt = o.address_interrupt,
                            .stop_interrupt = o.stop_interrupt,
                            .dual = on_dual_pins,
                            .dual_input_level = o.input_level,
                            .dual_sda_hold = o.sda_hold,
                            .dual_fm_plus = o.fm_plus},
                           clk_per_hz_);
        }
    }

    // ---- the protocol surface (29.3.2.3) ---------------------------------

    /// Is an address packet or a STOP waiting (APIF)?
    static bool addressed() { return T::address_flag(); }
    /// Is a data byte waiting, or the last one finished (DIF)?
    static bool data_ready() { return T::data_flag(); }
    /// What the last address packet asked for: true = the host READS.
    static bool host_reading() { return T::host_reading(); }
    /// What raised APIF: an address match (true) or a STOP (false).
    static bool matched_address() { return T::address_match(); }
    /// The address byte the match logic stored (SDATA after a match) -
    /// with PMEN or a mask this is the only way to learn WHICH address
    /// was called. Reading it also clears DIF, so read it right after
    /// the address interrupt and before responding.
    static uint8_t last_address() { return static_cast<uint8_t>(T::client_read() >> 1); }

    /// S1/S2: answer the address packet. ACK and go on (`ack`), or NACK
    /// it and wait for the next START.
    static void respond(TwiAck ack = TwiAck::ack) {
        T::client_command(ack == TwiAck::ack ? TwiClientCmd::response : TwiClientCmd::complete,
                          ack);
    }
    /// S3: end the transaction - execute the acknowledge action and wait
    /// for any START. Also the way to clear a STOP's APIF.
    static void complete(TwiAck ack = TwiAck::ack) {
        T::client_command(TwiClientCmd::complete, ack);
    }

    /// S1: take the received byte and answer it. In Smart mode the read
    /// of SDATA sends the acknowledge action by itself, so ACKACT is
    /// armed first and no command follows.
    static uint8_t receive(TwiAck ack = TwiAck::ack) {
        if (smart_) {
            T::client_ack_action(ack);
            return T::client_read();
        }
        const uint8_t v = T::client_read();
        T::client_command(ack == TwiAck::ack ? TwiClientCmd::response : TwiClientCmd::complete,
                          ack);
        return v;
    }

    /// S2: put the next byte on the bus. In Smart mode the write of
    /// SDATA continues the operation by itself.
    static void transmit(uint8_t v) {
        T::client_write(v);
        if (!smart_) T::client_command(TwiClientCmd::response);
    }

    /// The last acknowledge bit the HOST sent (valid after a transmit):
    /// true = NACK, the host wants no more.
    static bool nacked() { return T::client_rx_nack(); }
    /// S4: this client could not drive a high bit or a NACK. Cleared by
    /// any START, or here.
    static bool collision() { return T::collision(); }
    static void clear_collision() { T::clear_client_flags(TWI_COLL_bm); }
    static bool bus_error() { return T::client_bus_error(); }
    static void clear_bus_error() { T::clear_client_flags(TWI_BUSERR_bm); }

    /// Wait (bounded) for the next address packet or STOP.
    static bool wait_address(uint32_t spins = 500'000u) {
        for (;;) {
            if (T::address_flag()) return true;
            if (spins-- == 0) return false;
        }
    }
    /// Wait (bounded) for the next data byte / completed transmit.
    static bool wait_data(uint32_t spins = 500'000u) {
        for (;;) {
            if (T::data_flag()) return true;
            if (spins-- == 0) return false;
        }
    }

    static void enable_data_interrupt(bool on) { T::enable_data_interrupt(on); }
    static void enable_address_interrupt(bool on) { T::enable_address_interrupt(on); }
    static void enable_stop_interrupt(bool on) { T::enable_stop_interrupt(on); }

    /// The address-match space, changeable while enabled.
    static void address(uint8_t addr7, bool general_call = false) {
        T::client_address(addr7, general_call);
    }
    static void address_mask(uint8_t mask7) { T::address_mask(mask7); }
    static void second_address(uint8_t addr7) { T::second_address(addr7); }
    static void promiscuous(bool on) { T::promiscuous(on); }

    /// ISR body (TWIn_TWIS_vect). Clears nothing: DIF and APIF clear
    /// when the handler responds (SCMD) or touches SDATA.
    [[gnu::always_inline]] static typename T::ClientIsr isr() { return T::take_client(); }

    /// ClockUser: a client derives no rate from CLK_PER - the host does
    /// the clocking - but the bus error detector and the Stop interrupt
    /// need CLK_PER >= 4 x f_SCL (29.5.12, 29.5.10), so the peripheral
    /// clock is kept for that readback. Nothing is reprogrammed.
    static void rebase(uint32_t hz) { clk_per_hz_ = hz; }
    static uint32_t max_scl_hz() { return clk_per_hz_ / 4u; }
    /// Can this client see bus errors and STOPs at this SCL?
    static bool can_follow(uint32_t scl_hz) {
        return clk_per_hz_ != 0 && scl_hz <= clk_per_hz_ / 4u;
    }

    static void release() { T::release(); }

private:
    static void apply_address(const Options& o) {
        T::client_address(o.address, o.general_call);
        if (o.second_address) T::second_address(o.address_mask);
        else T::address_mask(o.address_mask);
    }
    static void apply_client_bits(const Options& o) {
        T::promiscuous(o.promiscuous);
        T::client_smart(o.smart);
        T::enable_data_interrupt(o.data_interrupt);
        T::enable_address_interrupt(o.address_interrupt);
        T::enable_stop_interrupt(o.stop_interrupt);
    }

    static inline uint32_t clk_per_hz_ = 0;
    static inline bool smart_ = false;
};

static_assert(ClockUser<TwiHost<0>>);
static_assert(ClockUser<TwiClient<0>>);

} // namespace brio
