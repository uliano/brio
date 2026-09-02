/*
 * i2c.hpp
 *
 * The SAM C21 SERCOM in I2C mode (DS60001479M ch. 33), in the strata
 * the rest of this target uses:
 *
 *  I2cm<n>     the HOST resource - a typed view of one instance's I2C
 *              host register set: the bus state machine, the baud
 *              generator with the chapter's own rise-time arithmetic,
 *              MB/SB/ERROR, the command strobes, the SYSOP
 *              synchronization discipline.
 *
 *  I2cs<n>     the CLIENT resource - the OTHER register set (unlike the
 *              SPI's two views these really differ: no BAUD, no bus
 *              state, AMATCH/DRDY/PREC instead of MB/SB): address
 *              matching in its three modes, the general call, the
 *              stretch-and-command surface.
 *
 *  I2cHost<n, pads>
 *              the TASK util/i2c_bus.hpp (= util/bus_master.hpp) drives:
 *              the transfer ENGINE. A Request is one BUS TENURE - write,
 *              read, or write-then-read joined by a repeated START - the
 *              SAME descriptor shape avrdx/twi.hpp carries, buffers as
 *              Lease::reply loans, completion as the bus AO's
 *              TransferDone with the i2c_* status vocabulary.
 *
 *  I2cClient<n, pads>
 *              the polled surface plus the ISR body for the other end of
 *              the wire; a client is a protocol and the protocol is the
 *              application's (the SpiClient position).
 *
 * PADS ARE FIXED BY THE CHAPTER: SDA is PAD[0] and SCL is PAD[1] (33.4),
 * so I2cPads carries only the two pins - there is no DOPO here. WHICH
 * PINS MAY CARRY I2C AT ALL is table 6-7's per-package list (on the
 * 64-pin J: PA08/09, PA12/13, PA16/17, PA22/23, PB12/13, PB16/17,
 * PB30/31) and NO device-header symbol encodes it, so it stays a stated
 * obligation on the pad pins - the one legality question this header
 * cannot ask the header.
 *
 * Facts that shape the code (33.5.x, 33.6.x, 33.8.x, 33.10.x, and
 * errata DS80000740S at silicon rev F):
 *  - CTRLA (but ENABLE/SWRST), CTRLB (but ACKACT/CMD), BAUD and the
 *    client's ADDR are ENABLE-PROTECTED (33.6.2.1): a write while
 *    enabled is DISCARDED. Everything here configures disabled;
 *  - the host's SYNCBUSY has a THIRD bit beside SWRST/ENABLE: SYSOP,
 *    raised by writing CTRLB.CMD, STATUS.BUSSTATE, ADDR or DATA while
 *    enabled (33.6.6). Every such store here WAITS FIRST (the sdadc
 *    discipline - wait before storing, never after);
 *  - the bus state machine (33.6.2.3) boots UNKNOWN and leaves it only
 *    by seeing a Stop, by an INACTOUT time-out, or by software forcing
 *    IDLE - so a host on a quiet bus that never forces IDLE never
 *    starts. force_idle() is therefore part of the host's init;
 *  - MB and SB hold SCL LOW (STATUS.CLKHOLD) until software answers
 *    with DATA, ADDR, a command, or a flag clear (33.10.6) - the
 *    unlimited-time-to-respond design the AVR's TWI shares;
 *  - writing ADDR.ADDR clears BUSERR, ARBLOST, LENERR and the time-out
 *    flags AUTOMATICALLY (33.10.7), which is why the engine starts a
 *    tenure with no clear ceremony at all;
 *  - ERRATUM 1.17.8 (LIVE): STATUS.CLKHOLD is documented read-only and
 *    IS WRITABLE, and writing it corrupts the clock-hold state. Every
 *    STATUS store in this file is a MASK THAT EXCLUDES BIT 7 by
 *    construction (i2cm_status_w1c / i2cs_status_w1c);
 *  - ERRATUM 1.17.10 (LIVE): 10-bit addressing in CLIENT mode is not
 *    functional, no workaround. I2csConfig has no ten-bit knob and the
 *    resource refuses ADDR.TENBITEN by construction (host-side 10-bit
 *    exists at the RESOURCE level; the engine speaks 7-bit, the avrdx
 *    Request shape);
 *  - ERRATUM 1.17.11 (LIVE): the client's error status bits are NOT
 *    cleared when INTFLAG.AMATCH is cleared, against 33.8.6. The
 *    workaround is code: I2cs<n>::clear_errors() writes them by hand
 *    and the client task spends it on every address match;
 *  - ERRATUM 1.17.13 (LIVE): Quick Command with SCLSM = 1 raises a bus
 *    error on a repeated start. i2cm_config_valid() refuses the pair;
 *  - ERRATUM 1.17.16: SWRST claimed non-functional while ENABLE = 0 on
 *    every revision. NOT REPRODUCED in SPI mode on this silicon
 *    (test_samc_spi a); the enable-first discipline is kept here too
 *    and the I2C-mode disposition is the suite's to measure;
 *  - ERRATUM 1.17.21 (LIVE): automatic address acknowledge (AACKEN)
 *    breaks on a repeated start, workaround "do not use the AACKEN
 *    feature". This driver has NO AACKEN knob at all - an AMATCH
 *    handler is the shape, and the workaround says so;
 *  - ERRATUM 1.17.22 (LIVE): the client's STATUS.RXNACK is INVALID at
 *    the first DRDY of a tenure. The workaround is a software flag
 *    armed at AMATCH; I2cClient carries it (first_drdy()) so the app
 *    does not have to rediscover the rule;
 *  - errata 1.17.6/1.17.7/1.17.9 cripple repeated starts in 10-bit and
 *    High-speed operation. The engine's only repeated start is the
 *    7-bit write-to-read turn, which none of the three touches. HS
 *    (CTRLA.SPEED = 0x2) is refused by i2cm_config_valid(): both its
 *    repeated-start halves are broken by errata with no workaround,
 *    and this desk's 1.5k breadboard bus is no HS bus anyway - a
 *    deliberate, stated non-feature;
 *  - the three SMBus time-outs count GCLK_SERCOM_SLOW, which must run
 *    at 32.768 kHz (33.6.3.1) and is SHARED by SERCOM0..4 (one
 *    channel, 18). Enabling a time-out without routing that channel
 *    hangs nothing but times nothing either; the config states it.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "sam.h"

#include "samc/clock.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/sercom.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

// =============================================================================
// The knobs (33.8.1, 33.10.1..3)
// =============================================================================

/// The bus speed a tenure runs at - the same three-step vocabulary
/// avrdx/twi.hpp's TwiSpeed speaks, because they name the same I2C
/// specification classes. High-speed (3.4 MHz) is deliberately absent:
/// errata 1.17.7 and 1.17.9 break its repeated starts with no
/// workaround (see the header comment).
enum class I2cSpeed : uint8_t {
    standard_100k,   ///< Sm, CTRLA.SPEED = 0x0
    fast_400k,       ///< Fm, CTRLA.SPEED = 0x0 (the same code serves both)
    fast_plus_1m,    ///< Fm+, CTRLA.SPEED = 0x1
};

constexpr uint32_t i2c_speed_hz(I2cSpeed s) {
    switch (s) {
        case I2cSpeed::standard_100k: return 100'000UL;
        case I2cSpeed::fast_400k: return 400'000UL;
        default: return 1'000'000UL;
    }
}
constexpr uint8_t i2c_speed_field(I2cSpeed s) {
    return s == I2cSpeed::fast_plus_1m
               ? static_cast<uint8_t>(SERCOM_I2CM_CTRLA_SPEED_FASTPLUS_MODE_Val)
               : static_cast<uint8_t>(SERCOM_I2CM_CTRLA_SPEED_STANDARD_AND_FAST_MODE_Val);
}

/// CTRLA.SDAHOLD - the SDA hold time after an SCL falling edge, the
/// SMBus-compatibility knob (33.8.1). The DA errata's swapped-encoding
/// trap does NOT exist here: these are the header's own codes.
enum class I2cSdaHold : uint8_t {
    off = SERCOM_I2CM_CTRLA_SDAHOLD_DISABLE_Val,
    ns75 = SERCOM_I2CM_CTRLA_SDAHOLD_75NS_Val,
    ns450 = SERCOM_I2CM_CTRLA_SDAHOLD_450NS_Val,
    ns600 = SERCOM_I2CM_CTRLA_SDAHOLD_600NS_Val,
};

/// CTRLA.INACTOUT, HOST only: the inactive-bus time-out that also
/// unsticks the bus state machine from BUSY (33.6.2.3). The
/// microsecond names hold at 100 kHz - the field counts SCL cycles.
enum class I2cInactiveTimeout : uint8_t {
    disabled = SERCOM_I2CM_CTRLA_INACTOUT_DISABLE_Val,
    us55 = SERCOM_I2CM_CTRLA_INACTOUT_55US_Val,
    us105 = SERCOM_I2CM_CTRLA_INACTOUT_105US_Val,
    us205 = SERCOM_I2CM_CTRLA_INACTOUT_205US_Val,
};

/// The client's CTRLB.AMODE (33.8.2). 0x3 is Reserved and refused.
enum class I2cAddressMode : uint8_t {
    mask = 0x0,           ///< ADDR masked by ADDRMASK (0 bits = don't care)
    two_addresses = 0x1,  ///< ADDR and ADDRMASK are two exact addresses
    range = 0x2,          ///< every address in [ADDRMASK, ADDR]
};

/// STATUS.BUSSTATE, the host's four bus states (33.6.2.3).
enum class I2cBusState : uint8_t {
    unknown = 0x0,
    idle = 0x1,
    owner = 0x2,
    busy = 0x3,
};

// =============================================================================
// Pads and pins
// =============================================================================

/**
 * SDA and SCL are PAD[0] and PAD[1], fixed by the chapter (33.4) -
 * there is nothing to choose but the PINS, stated exactly as UartPads
 * and SpiPads state theirs. That the pins are on table 6-7's
 * I2C-capable list is the caller's obligation: no header symbol
 * encodes it (the bench pair PA22/PA23 is on the list).
 */
struct I2cPads {
    SercomPadPin sda_pin{};   ///< PAD[0]
    SercomPadPin scl_pin{};   ///< PAD[1]
};

constexpr bool i2c_pads_valid(const I2cPads& p) {
    return p.sda_pin.valid() && p.scl_pin.valid();
}

// =============================================================================
// The baud arithmetic (pure: no register is touched below this line)
// =============================================================================

/**
 * The chapter's own formula (33.6.2.4.1), solved for the register:
 *
 *     f_SCL = f_GCLK / (10 + BAUD + BAUDLOW + f_GCLK x T_RISE)
 *
 * with BAUD timing the HIGH half and BAUDLOW the LOW half (BAUDLOW = 0
 * makes BAUD time both). T_RISE is the BUS'S, not the chip's - the
 * pull-ups and the wire capacitance own it, which is why it is an
 * ARGUMENT here exactly as it is in avrdx/twi.hpp's three-step: a baud
 * computed with a rise time the bus does not have lands the SCL low
 * time below the specification floor by exactly the difference.
 *
 * THE SPLIT IS THE SPECIFICATION'S: Fm+ requires a nominal 1:2
 * high-to-low ratio (the chapter's own note), Sm/Fm are symmetric. The
 * division rounds so the produced rate is never ABOVE the request - a
 * requested SCL is a bus ceiling.
 */
struct I2cBaud {
    uint8_t baud = 0;      ///< BAUD.BAUD - times T_HIGH
    uint8_t baudlow = 0;   ///< BAUD.BAUDLOW - times T_LOW when nonzero
};

constexpr std::optional<I2cBaud> i2c_baud_for(uint32_t gclk_hz, uint32_t scl_hz,
                                              uint32_t rise_ns, bool fast_plus) {
    if (gclk_hz == 0u || scl_hz == 0u) {
        return {};
    }
    // f_GCLK x T_RISE in cycles, computed without overflow: gclk_hz up
    // to 48 MHz and rise_ns bounded by the I2C spec's 1000 ns keep the
    // product inside 64 bits; the result is a small integer.
    const uint32_t rise_cycles =
        static_cast<uint32_t>((static_cast<uint64_t>(gclk_hz) * rise_ns) / 1'000'000'000ULL);
    const uint32_t total = (gclk_hz + scl_hz - 1u) / scl_hz;   // ceil: never faster
    if (total < 10u + rise_cycles + 1u) {
        return {};   // the generator cannot go that fast at this clock
    }
    const uint32_t k = total - 10u - rise_cycles;   // BAUD + BAUDLOW budget
    uint32_t high = 0;
    uint32_t low = 0;
    if (fast_plus) {
        // 1:2 high:low. low = ceil(2k/3) keeps the LOW half - the one
        // the specification floors - at or above its share.
        low = (2u * k + 2u) / 3u;
        high = k - low;
        if (high == 0u) {
            high = 1u;
            low = k - 1u;
        }
    } else {
        high = k / 2u;
        low = k - high;   // the odd cycle lands in LOW, never in HIGH
        if (high == low) {
            low = 0u;     // symmetric: let BAUD time both halves
            high = k / 2u;
            if (2u * high < k) {
                low = high + 1u;   // odd k: asymmetric by one, LOW longer
            }
        }
    }
    if (high == 0u || high > 255u || low > 255u) {
        return {};   // outside the two eight-bit fields
    }
    return I2cBaud{static_cast<uint8_t>(high), static_cast<uint8_t>(low)};
}

/// What SCL a register pair really produces at gclk_hz on a bus with
/// this rise time (the readback half, actual_scl in the AVR's spelling).
constexpr uint32_t i2c_scl_hz(uint32_t gclk_hz, I2cBaud b, uint32_t rise_ns) {
    const uint32_t rise_cycles =
        static_cast<uint32_t>((static_cast<uint64_t>(gclk_hz) * rise_ns) / 1'000'000'000ULL);
    const uint32_t low = b.baudlow != 0u ? b.baudlow : b.baud;
    const uint32_t total = 10u + b.baud + low + rise_cycles;
    return total != 0u ? gclk_hz / total : 0u;
}

// =============================================================================
// Status W1C masks - erratum 1.17.8 as construction
// =============================================================================

/// The HOST STATUS bits that are legal to write one to. CLKHOLD (bit 7)
/// is deliberately ABSENT: the erratum makes it writable against the
/// datasheet and writing it corrupts the clock-hold state, so no mask
/// in this file can even express it. BUSSTATE is a FIELD, not a flag -
/// it travels through force_idle() alone.
struct I2cmStatus {
    I2cmStatus() = delete;
    static constexpr uint16_t bus_error = SERCOM_I2CM_STATUS_BUSERR_Msk;
    static constexpr uint16_t arb_lost = SERCOM_I2CM_STATUS_ARBLOST_Msk;
    static constexpr uint16_t low_timeout = SERCOM_I2CM_STATUS_LOWTOUT_Msk;
    static constexpr uint16_t len_error = SERCOM_I2CM_STATUS_LENERR_Msk;
    static constexpr uint16_t sext_timeout = SERCOM_I2CM_STATUS_SEXTTOUT_Msk;
    static constexpr uint16_t mext_timeout = SERCOM_I2CM_STATUS_MEXTTOUT_Msk;
    static constexpr uint16_t w1c_all =
        bus_error | arb_lost | low_timeout | len_error | sext_timeout | mext_timeout;
};

/// The CLIENT STATUS W1C set - the erratum-1.17.11 list as it exists in
/// this register (33.8.6): COLL, BUSERR, LOWTOUT, SEXTTOUT, HS. Same
/// CLKHOLD exclusion, same reason.
struct I2csStatus {
    I2csStatus() = delete;
    static constexpr uint16_t bus_error = SERCOM_I2CS_STATUS_BUSERR_Msk;
    static constexpr uint16_t collision = SERCOM_I2CS_STATUS_COLL_Msk;
    static constexpr uint16_t low_timeout = SERCOM_I2CS_STATUS_LOWTOUT_Msk;
    static constexpr uint16_t sext_timeout = SERCOM_I2CS_STATUS_SEXTTOUT_Msk;
    static constexpr uint16_t high_speed = SERCOM_I2CS_STATUS_HS_Msk;
    static constexpr uint16_t w1c_all =
        bus_error | collision | low_timeout | sext_timeout | high_speed;
};

struct I2cmFlag {
    I2cmFlag() = delete;
    static constexpr uint8_t mb = SERCOM_I2CM_INTFLAG_MB_Msk;
    static constexpr uint8_t sb = SERCOM_I2CM_INTFLAG_SB_Msk;
    static constexpr uint8_t error = SERCOM_I2CM_INTFLAG_ERROR_Msk;
    static constexpr uint8_t all = mb | sb | error;
};

struct I2csFlag {
    I2csFlag() = delete;
    static constexpr uint8_t stop = SERCOM_I2CS_INTFLAG_PREC_Msk;
    static constexpr uint8_t amatch = SERCOM_I2CS_INTFLAG_AMATCH_Msk;
    static constexpr uint8_t drdy = SERCOM_I2CS_INTFLAG_DRDY_Msk;
    static constexpr uint8_t error = SERCOM_I2CS_INTFLAG_ERROR_Msk;
    static constexpr uint8_t all = stop | amatch | drdy | error;
};

// =============================================================================
// The two configurations
// =============================================================================

/// The HOST configuration. `baud` comes from i2c_baud_for() - the
/// resource speaks the register, the task speaks hertz and rise time.
struct I2cmConfig {
    I2cPads pads{};
    I2cSpeed speed = I2cSpeed::standard_100k;
    I2cBaud baud{};
    I2cSdaHold sda_hold = I2cSdaHold::off;
    /// Also the BUSY-state escape of 33.6.2.3; without it a bus left
    /// BUSY by a glitch stays BUSY until a real Stop.
    I2cInactiveTimeout inactive_timeout = I2cInactiveTimeout::disabled;
    /// CTRLA.SCLSM. The engine runs SCLSM = 0 (stretch before the ACK,
    /// DATA in hand before the acknowledge decision). Erratum 1.17.13
    /// refuses SCLSM = 1 with quick command.
    bool scl_stretch_after_ack = false;
    /// CTRLB.SMEN: an ACK/NACK (per ACKACT) fires on every DATA read.
    bool smart = false;
    /// CTRLB.QCEN (33.6.3.4).
    bool quick_command = false;
    /// The three SMBus time-outs (33.6.3.1). ALL count the SHARED
    /// GCLK_SERCOM_SLOW channel, which must be routed to a 32.768 kHz
    /// generator by the caller - this driver cannot know which
    /// generator carries 32 kHz, and the channel is SERCOM0..4's
    /// collectively (Sercom<n>::gclk_slow_id()).
    bool scl_low_timeout = false;    ///< CTRLA.LOWTOUTEN, T_TIMEOUT 25..35 ms
    bool client_extend_timeout = false;   ///< CTRLA.SEXTTOEN, T_LOW:SEXT 25 ms
    bool host_extend_timeout = false;     ///< CTRLA.MEXTTOEN, T_LOW:MEXT 10 ms
    bool run_standby = false;   ///< CTRLA.RUNSTDBY
    bool debug_stop = false;    ///< DBGCTRL.DBGSTOP
};

/// The CLIENT configuration. NO ten-bit knob (erratum 1.17.10: not
/// functional, no workaround) and NO automatic-acknowledge knob
/// (erratum 1.17.21: broken on repeated start, workaround "implement an
/// AMATCH handler" - which is exactly what I2cClient is).
struct I2csConfig {
    I2cPads pads{};
    /// The three-mode address recognition (33.8.2). In `mask` mode
    /// `second` is the mask (0 = exact); in `two_addresses` it is the
    /// second address; in `range` the LOWER bound with `address` the
    /// upper.
    I2cAddressMode address_mode = I2cAddressMode::mask;
    uint8_t address = 0;   ///< 7-bit, unshifted
    uint8_t second = 0;    ///< mask / second address / lower bound
    bool general_call = false;   ///< ADDR.GENCEN (33.8.8)
    I2cSpeed speed = I2cSpeed::standard_100k;
    I2cSdaHold sda_hold = I2cSdaHold::off;
    bool scl_stretch_after_ack = false;   ///< CTRLA.SCLSM
    bool smart = false;                   ///< CTRLB.SMEN
    /// CTRLB.GCMD - PREC on a Stop only if addressed since the last one
    /// (33.6.2.5.6).
    bool group_command = false;
    bool scl_low_timeout = false;         ///< CTRLA.LOWTOUTEN (shared SLOW clock)
    bool client_extend_timeout = false;   ///< CTRLA.SEXTTOEN
    bool run_standby = false;   ///< CTRLA.RUNSTDBY: the address-match wake
    // NO debug_stop here: the CLIENT register map has no DBGCTRL (33.7
    // ends at DATA; the device header agrees) - the knob is the host
    // view's alone.
};

constexpr bool i2c_speed_valid(I2cSpeed s) {
    return s == I2cSpeed::standard_100k || s == I2cSpeed::fast_400k ||
           s == I2cSpeed::fast_plus_1m;
}
constexpr bool i2c_address_mode_valid(I2cAddressMode m) {
    return m == I2cAddressMode::mask || m == I2cAddressMode::two_addresses ||
           m == I2cAddressMode::range;
}

constexpr bool i2cm_config_valid(const I2cmConfig& c) {
    if (!i2c_pads_valid(c.pads)) return false;
    if (!i2c_speed_valid(c.speed)) return false;               // HS refused with the enum
    // silicon (erratum 1.17.13): quick command + SCLSM = 1 raises a bus
    // error on every repeated start.
    if (c.quick_command && c.scl_stretch_after_ack) return false;
    // driver: a BAUD of all zeros is the reset value, not a rate - the
    // chapter's own note requires BAUD and/or BAUDLOW nonzero.
    if (c.baud.baud == 0u && c.baud.baudlow == 0u) return false;
    return true;
}

constexpr bool i2cs_config_valid(const I2csConfig& c) {
    if (!i2c_pads_valid(c.pads)) return false;
    if (!i2c_speed_valid(c.speed)) return false;
    if (!i2c_address_mode_valid(c.address_mode)) return false;
    // silicon: 7-bit addresses.
    if ((c.address & 0x80u) != 0u || (c.second & 0x80u) != 0u) return false;
    // driver: a range with the bounds inverted matches nothing 33.8.2
    // describes.
    if (c.address_mode == I2cAddressMode::range && c.second > c.address) return false;
    return true;
}

// =============================================================================
// The host resource
// =============================================================================

template <uint8_t n>
class I2cm {
    using Base = Sercom<n>;

public:
    I2cm() = delete;

    static constexpr uint8_t index = n;

    static sercom_i2cm_registers_t& regs() { return Base::i2cm_regs(); }
    static constexpr uint8_t gclk_core_id() { return Base::gclk_core_id(); }
    static constexpr uint8_t gclk_slow_id() { return Base::gclk_slow_id(); }
    static constexpr uint32_t apb_mask() { return Base::apb_mask(); }
    static constexpr IRQn_Type irq() { return Base::irq(); }
    static constexpr uint8_t dma_rx_trigger() { return Base::dma_rx_trigger(); }
    static constexpr uint8_t dma_tx_trigger() { return Base::dma_tx_trigger(); }

    static void bus_clock(bool on) { Base::bus_clock(on); }
    static bool core_clock(uint8_t generator) { return Base::core_clock(generator); }

    // ---- synchronization (33.6.6) -------------------------------------------

    static bool sync_busy(uint32_t mask) { return (regs().SERCOM_SYNCBUSY & mask) != 0u; }
    static bool wait_sync(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().SERCOM_SYNCBUSY, mask, false, spins);
    }
    /// The one that matters per-operation: CMD, BUSSTATE, ADDR and DATA
    /// all synchronize through SYSOP while enabled. WAIT BEFORE STORING.
    static bool wait_sysop(uint32_t spins = 0xFFFFu) {
        return wait_sync(SERCOM_I2CM_SYNCBUSY_SYSOP_Msk, spins);
    }

    // ---- reset and enable ---------------------------------------------------

    static bool enabled() {
        return (regs().SERCOM_CTRLA & SERCOM_I2CM_CTRLA_ENABLE_Msk) != 0u;
    }

    /// The spi.hpp discipline, for the same erratum (1.17.16): a
    /// disabled instance is enabled first - in HOST mode, whose clock is
    /// the GCLK this init routes - and reset from there.
    static bool reset(uint32_t spins = 0xFFFFu) {
        if (!enabled()) {
            regs().SERCOM_CTRLA =
                SERCOM_I2CM_CTRLA_MODE(SERCOM_I2CM_CTRLA_MODE_I2C_MASTER_Val) |
                SERCOM_I2CM_CTRLA_ENABLE_Msk;
            if (!wait_sync(SERCOM_I2CM_SYNCBUSY_ENABLE_Msk, spins)) {
                return false;
            }
        }
        regs().SERCOM_CTRLA = SERCOM_I2CM_CTRLA_SWRST_Msk;
        return wait_sync(SERCOM_I2CM_SYNCBUSY_SWRST_Msk, spins);
    }

    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint32_t v = regs().SERCOM_CTRLA;
        regs().SERCOM_CTRLA = on ? (v | SERCOM_I2CM_CTRLA_ENABLE_Msk)
                                 : (v & ~SERCOM_I2CM_CTRLA_ENABLE_Msk);
        return wait_sync(SERCOM_I2CM_SYNCBUSY_ENABLE_Msk, spins);
    }

    // ---- configuration ------------------------------------------------------

    static bool configure(const I2cmConfig& c, uint32_t spins = 0xFFFFu) {
        if (!i2cm_config_valid(c)) {
            return false;
        }
        if (!enable(false, spins)) {
            return false;
        }
        regs().SERCOM_INTENCLR = I2cmFlag::all;
        regs().SERCOM_CTRLA =
            SERCOM_I2CM_CTRLA_MODE(SERCOM_I2CM_CTRLA_MODE_I2C_MASTER_Val) |
            SERCOM_I2CM_CTRLA_SDAHOLD(static_cast<uint32_t>(c.sda_hold)) |
            SERCOM_I2CM_CTRLA_INACTOUT(static_cast<uint32_t>(c.inactive_timeout)) |
            SERCOM_I2CM_CTRLA_SPEED(i2c_speed_field(c.speed)) |
            (c.scl_stretch_after_ack ? SERCOM_I2CM_CTRLA_SCLSM_Msk : 0u) |
            (c.scl_low_timeout ? SERCOM_I2CM_CTRLA_LOWTOUTEN_Msk : 0u) |
            (c.client_extend_timeout ? SERCOM_I2CM_CTRLA_SEXTTOEN_Msk : 0u) |
            (c.host_extend_timeout ? SERCOM_I2CM_CTRLA_MEXTTOEN_Msk : 0u) |
            (c.run_standby ? SERCOM_I2CM_CTRLA_RUNSTDBY_Msk : 0u);
        regs().SERCOM_CTRLB =
            (c.smart ? SERCOM_I2CM_CTRLB_SMEN_Msk : 0u) |
            (c.quick_command ? SERCOM_I2CM_CTRLB_QCEN_Msk : 0u);
        regs().SERCOM_BAUD = SERCOM_I2CM_BAUD_BAUD(c.baud.baud) |
                             SERCOM_I2CM_BAUD_BAUDLOW(c.baud.baudlow);
        regs().SERCOM_DBGCTRL =
            c.debug_stop ? static_cast<uint8_t>(SERCOM_I2CM_DBGCTRL_DBGSTOP_Msk) : 0u;
        return true;
    }

    /// The compile-time twin.
    template <I2cmConfig cfg>
    static bool configure(uint32_t spins = 0xFFFFu) {
        static_assert(i2cm_config_valid(cfg),
                      "brio I2cm: this host configuration is refused - see "
                      "i2cm_config_valid() (High-speed and quick-command-with-SCLSM "
                      "are errata refusals, a zero BAUD is the chapter's own note)");
        return configure(cfg, spins);
    }

    static uint32_t ctrla() { return regs().SERCOM_CTRLA; }
    static uint32_t ctrlb() { return regs().SERCOM_CTRLB; }
    static uint32_t baud_reg() { return regs().SERCOM_BAUD; }
    static uint8_t dbgctrl() { return regs().SERCOM_DBGCTRL; }

    // ---- the bus state machine (33.6.2.3) -----------------------------------

    static I2cBusState bus_state() {
        return static_cast<I2cBusState>((regs().SERCOM_STATUS &
                                         SERCOM_I2CM_STATUS_BUSSTATE_Msk) >>
                                        SERCOM_I2CM_STATUS_BUSSTATE_Pos);
    }

    /// UNKNOWN -> IDLE, the software escape the chapter requires after
    /// every enable on a quiet bus (only a Stop or an INACTOUT time-out
    /// gets there otherwise). BUSSTATE is write-synchronized: the store
    /// raises SYSOP and the wait is spent HERE, so the caller's next
    /// ADDR write finds the machine settled.
    static bool force_idle(uint32_t spins = 0xFFFFu) {
        if (!wait_sysop(spins)) {
            return false;
        }
        regs().SERCOM_STATUS = SERCOM_I2CM_STATUS_BUSSTATE(
            static_cast<uint16_t>(I2cBusState::idle));
        if (!wait_sysop(spins)) {
            return false;
        }
        return bus_state() == I2cBusState::idle;
    }

    // ---- status and flags ---------------------------------------------------

    [[gnu::always_inline]] static uint16_t status() { return regs().SERCOM_STATUS; }
    /// W1C, THROUGH THE MASK THAT CANNOT NAME CLKHOLD (erratum 1.17.8).
    static void clear_status(uint16_t mask) {
        regs().SERCOM_STATUS = mask & I2cmStatus::w1c_all;
    }

    static bool rx_nack() { return (status() & SERCOM_I2CM_STATUS_RXNACK_Msk) != 0u; }
    static bool arb_lost() { return (status() & I2cmStatus::arb_lost) != 0u; }
    static bool bus_error() { return (status() & I2cmStatus::bus_error) != 0u; }
    static bool clock_hold() { return (status() & SERCOM_I2CM_STATUS_CLKHOLD_Msk) != 0u; }

    [[gnu::always_inline]] static uint8_t pending() {
        return static_cast<uint8_t>(regs().SERCOM_INTFLAG & regs().SERCOM_INTENSET);
    }
    static uint8_t flags() { return regs().SERCOM_INTFLAG; }
    static void clear_flags(uint8_t mask) { regs().SERCOM_INTFLAG = mask; }
    static void enable_interrupt(uint8_t mask, bool on) {
        if (on) {
            regs().SERCOM_INTENSET = mask;
        } else {
            regs().SERCOM_INTENCLR = mask;
        }
    }

    // ---- the operations (33.6.2.4.x) ----------------------------------------

    /// Start (or repeated-start) a tenure: address + direction bit.
    /// Writing ADDR auto-clears BUSERR/ARBLOST/LENERR and the time-out
    /// flags (33.10.7/33.10.9) - no ceremony before it. The wait is
    /// BEFORE the store (SYSOP).
    static bool start_address(uint8_t addr7, bool read, uint32_t spins = 0xFFFFu) {
        if (!wait_sysop(spins)) {
            return false;
        }
        regs().SERCOM_ADDR = SERCOM_I2CM_ADDR_ADDR(
            (static_cast<uint32_t>(addr7) << 1) | (read ? 1u : 0u));
        return true;
    }

    /// One command strobe with its acknowledge action (33.10.2): the two
    /// can go in one store and the action precedes the command.
    static bool command(uint8_t cmd, bool nack, uint32_t spins = 0xFFFFu) {
        if (!wait_sysop(spins)) {
            return false;
        }
        regs().SERCOM_CTRLB = (regs().SERCOM_CTRLB &
                               ~(SERCOM_I2CM_CTRLB_CMD_Msk | SERCOM_I2CM_CTRLB_ACKACT_Msk)) |
                              SERCOM_I2CM_CTRLB_CMD(cmd) |
                              (nack ? SERCOM_I2CM_CTRLB_ACKACT_Msk : 0u);
        return true;
    }
    static bool stop(bool nack = false, uint32_t spins = 0xFFFFu) {
        return command(0x3u, nack, spins);
    }
    static bool read_next(uint32_t spins = 0xFFFFu) { return command(0x2u, false, spins); }

    [[gnu::always_inline]] static uint8_t data() {
        return static_cast<uint8_t>(regs().SERCOM_DATA);
    }
    static bool data(uint8_t v, uint32_t spins = 0xFFFFu) {
        if (!wait_sysop(spins)) {
            return false;
        }
        regs().SERCOM_DATA = v;
        return true;
    }

    static void release(uint32_t spins = 0xFFFFu) {
        regs().SERCOM_INTENCLR = I2cmFlag::all;
        (void)enable(false, spins);
        GclkChannel::disconnect(gclk_core_id());
        bus_clock(false);
    }
};

// =============================================================================
// The client resource
// =============================================================================

template <uint8_t n>
class I2cs {
    using Base = Sercom<n>;

public:
    I2cs() = delete;

    static constexpr uint8_t index = n;

    static sercom_i2cs_registers_t& regs() { return Base::i2cs_regs(); }
    static constexpr uint8_t gclk_core_id() { return Base::gclk_core_id(); }
    static constexpr uint8_t gclk_slow_id() { return Base::gclk_slow_id(); }
    static constexpr uint32_t apb_mask() { return Base::apb_mask(); }
    static constexpr IRQn_Type irq() { return Base::irq(); }

    static void bus_clock(bool on) { Base::bus_clock(on); }
    static bool core_clock(uint8_t generator) { return Base::core_clock(generator); }

    static bool sync_busy(uint32_t mask) { return (regs().SERCOM_SYNCBUSY & mask) != 0u; }
    static bool wait_sync(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().SERCOM_SYNCBUSY, mask, false, spins);
    }

    static bool enabled() {
        return (regs().SERCOM_CTRLA & SERCOM_I2CS_CTRLA_ENABLE_Msk) != 0u;
    }

    /// Enable-first for the same erratum; the client's own clocking is
    /// the HOST'S SCL, but the enable synchronizes against the GCLK this
    /// init routes - so the brief host-mode enable of the reset is what
    /// guarantees a clock to synchronize against, exactly as in spi.hpp.
    static bool reset(uint32_t spins = 0xFFFFu) {
        if (!enabled()) {
            regs().SERCOM_CTRLA =
                SERCOM_I2CS_CTRLA_MODE(SERCOM_I2CS_CTRLA_MODE_I2C_MASTER_Val) |
                SERCOM_I2CS_CTRLA_ENABLE_Msk;
            if (!wait_sync(SERCOM_I2CS_SYNCBUSY_ENABLE_Msk, spins)) {
                return false;
            }
        }
        regs().SERCOM_CTRLA = SERCOM_I2CS_CTRLA_SWRST_Msk;
        return wait_sync(SERCOM_I2CS_SYNCBUSY_SWRST_Msk, spins);
    }

    static bool enable(bool on, uint32_t spins = 0xFFFFu) {
        const uint32_t v = regs().SERCOM_CTRLA;
        regs().SERCOM_CTRLA = on ? (v | SERCOM_I2CS_CTRLA_ENABLE_Msk)
                                 : (v & ~SERCOM_I2CS_CTRLA_ENABLE_Msk);
        return wait_sync(SERCOM_I2CS_SYNCBUSY_ENABLE_Msk, spins);
    }

    static bool configure(const I2csConfig& c, uint32_t spins = 0xFFFFu) {
        if (!i2cs_config_valid(c)) {
            return false;
        }
        if (!enable(false, spins)) {
            return false;
        }
        regs().SERCOM_INTENCLR = I2csFlag::all;
        regs().SERCOM_CTRLA =
            SERCOM_I2CS_CTRLA_MODE(SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE_Val) |
            SERCOM_I2CS_CTRLA_SDAHOLD(static_cast<uint32_t>(c.sda_hold)) |
            SERCOM_I2CS_CTRLA_SPEED(i2c_speed_field(c.speed)) |
            (c.scl_stretch_after_ack ? SERCOM_I2CS_CTRLA_SCLSM_Msk : 0u) |
            (c.scl_low_timeout ? SERCOM_I2CS_CTRLA_LOWTOUTEN_Msk : 0u) |
            (c.client_extend_timeout ? SERCOM_I2CS_CTRLA_SEXTTOEN_Msk : 0u) |
            (c.run_standby ? SERCOM_I2CS_CTRLA_RUNSTDBY_Msk : 0u);
        regs().SERCOM_CTRLB =
            SERCOM_I2CS_CTRLB_AMODE(static_cast<uint32_t>(c.address_mode)) |
            (c.smart ? SERCOM_I2CS_CTRLB_SMEN_Msk : 0u) |
            (c.group_command ? SERCOM_I2CS_CTRLB_GCMD_Msk : 0u);
        // ADDR is enable-protected IN CLIENT OPERATION (33.6.2.1) -
        // written here, disabled, with the general call in bit 0 and
        // NEVER TENBITEN (erratum 1.17.10: not functional).
        regs().SERCOM_ADDR =
            SERCOM_I2CS_ADDR_ADDR(c.address) |
            SERCOM_I2CS_ADDR_ADDRMASK(c.second) |
            (c.general_call ? SERCOM_I2CS_ADDR_GENCEN_Msk : 0u);
        // No DBGCTRL store: the client view has no such register.
        return true;
    }

    template <I2csConfig cfg>
    static bool configure(uint32_t spins = 0xFFFFu) {
        static_assert(i2cs_config_valid(cfg),
                      "brio I2cs: this client configuration is refused - see "
                      "i2cs_config_valid() (a Reserved AMODE, an 8-bit address, an "
                      "inverted range)");
        return configure(cfg, spins);
    }

    static uint32_t ctrla() { return regs().SERCOM_CTRLA; }
    static uint32_t ctrlb() { return regs().SERCOM_CTRLB; }
    static uint32_t addr_reg() { return regs().SERCOM_ADDR; }

    // ---- status and flags ---------------------------------------------------

    [[gnu::always_inline]] static uint16_t status() { return regs().SERCOM_STATUS; }
    /// W1C through the CLKHOLD-free mask (erratum 1.17.8 again).
    static void clear_status(uint16_t mask) {
        regs().SERCOM_STATUS = mask & I2csStatus::w1c_all;
    }
    /// ERRATUM 1.17.11's workaround, spelled once: the error bits that
    /// 33.8.6 promises AMATCH's clear will take with it, and does not.
    static void clear_errors() { regs().SERCOM_STATUS = I2csStatus::w1c_all; }

    /// Host read (the client transmits) or host write? Valid at AMATCH.
    static bool host_reads() { return (status() & SERCOM_I2CS_STATUS_DIR_Msk) != 0u; }
    /// Was this address match a REPEATED start? Valid only while AMATCH
    /// stands (33.8.6).
    static bool repeated_start() { return (status() & SERCOM_I2CS_STATUS_SR_Msk) != 0u; }
    static bool rx_nack() { return (status() & SERCOM_I2CS_STATUS_RXNACK_Msk) != 0u; }
    static bool collision() { return (status() & I2csStatus::collision) != 0u; }
    static bool clock_hold() { return (status() & SERCOM_I2CS_STATUS_CLKHOLD_Msk) != 0u; }

    [[gnu::always_inline]] static uint8_t pending() {
        return static_cast<uint8_t>(regs().SERCOM_INTFLAG & regs().SERCOM_INTENSET);
    }
    static uint8_t flags() { return regs().SERCOM_INTFLAG; }
    static void clear_flags(uint8_t mask) { regs().SERCOM_INTFLAG = mask; }
    static void enable_interrupt(uint8_t mask, bool on) {
        if (on) {
            regs().SERCOM_INTENSET = mask;
        } else {
            regs().SERCOM_INTENCLR = mask;
        }
    }

    // ---- the operations (33.6.2.5.x) ----------------------------------------

    /// Answer an address match: ACK or NACK through CTRLB.CMD = 0x3,
    /// whose acknowledge action follows STATUS.DIR by itself. The
    /// command clears AMATCH (and every other flag) on its own - and
    /// erratum 1.17.11's leftovers are swept first, so the next match
    /// starts clean.
    static void answer_address(bool ack) {
        clear_errors();
        regs().SERCOM_CTRLB = (regs().SERCOM_CTRLB &
                               ~(SERCOM_I2CS_CTRLB_CMD_Msk | SERCOM_I2CS_CTRLB_ACKACT_Msk)) |
                              SERCOM_I2CS_CTRLB_CMD(0x3u) |
                              (ack ? 0u : SERCOM_I2CS_CTRLB_ACKACT_Msk);
    }

    /// After a received data byte (DRDY, host write): take the byte and
    /// answer it. CMD = 0x3 continues the reception with the acknowledge
    /// action executed first.
    static uint8_t take(bool ack = true) {
        const uint8_t v = static_cast<uint8_t>(regs().SERCOM_DATA);
        regs().SERCOM_CTRLB = (regs().SERCOM_CTRLB &
                               ~(SERCOM_I2CS_CTRLB_CMD_Msk | SERCOM_I2CS_CTRLB_ACKACT_Msk)) |
                              SERCOM_I2CS_CTRLB_CMD(0x3u) |
                              (ack ? 0u : SERCOM_I2CS_CTRLB_ACKACT_Msk);
        return v;
    }

    /// After DRDY in a host READ: hand the next byte to the shifter.
    /// Writing DATA releases the stretch by itself (33.10.6's list
    /// holds for the client's DRDY too).
    static void give(uint8_t v) { regs().SERCOM_DATA = v; }

    /// CMD 0x2 (table 33-3): complete the transaction in response to a
    /// DRDY - in a host READ, after the host's closing NACK, this is
    /// what releases the machinery to "wait for any start" instead of
    /// stretching for a byte nobody wants; in a host WRITE it executes
    /// the acknowledge action first. The command clears every flag by
    /// itself, and 1.17.11's leftovers are swept like answer_address's.
    static void end_transaction(bool ack = true) {
        clear_errors();
        regs().SERCOM_CTRLB = (regs().SERCOM_CTRLB &
                               ~(SERCOM_I2CS_CTRLB_CMD_Msk | SERCOM_I2CS_CTRLB_ACKACT_Msk)) |
                              SERCOM_I2CS_CTRLB_CMD(0x2u) |
                              (ack ? 0u : SERCOM_I2CS_CTRLB_ACKACT_Msk);
    }

    [[gnu::always_inline]] static uint8_t data() {
        return static_cast<uint8_t>(regs().SERCOM_DATA);
    }

    static void release(uint32_t spins = 0xFFFFu) {
        regs().SERCOM_INTENCLR = I2csFlag::all;
        (void)enable(false, spins);
        GclkChannel::disconnect(gclk_core_id());
        bus_clock(false);
    }
};

// =============================================================================
// The host task - the engine util/i2c_bus.hpp drives
// =============================================================================

/*
 * I2cHost<n, pads, generator>
 *
 * The transfer engine, driven by util/bus_master.hpp exactly as
 * avrdx/twi.hpp's TwiHost is: a Request is ONE BUS TENURE - write, read,
 * or write-then-read joined by a repeated START (the register-access
 * idiom) - and the empty request is an address probe. ALWAYS
 * asynchronous: start() returns false and a TransferDone{status()}
 * follows from the ISR, the avrdx engine's own contract.
 *
 * The state machine is 33.6.2.4's, SCLSM = 0, smart mode off: MB is
 * "the host moved a byte out (or failed to)", SB is "a byte arrived",
 * and each holds SCL until the next operation - so the ISR body IS the
 * whole protocol, one interrupt per byte, nothing polled.
 *
 * Status vocabulary (util/i2c_bus.hpp): i2c_nack_addr when the address
 * byte went unacknowledged, i2c_nack_data for a data byte, i2c_arb_lost
 * when another host won the wire, i2c_bus_error for a protocol
 * violation. After arbitration is lost the silicon has already released
 * the bus and a Stop is NOT ours to send (33.6.2.4.2 case 1) - the
 * engine clears the flag and reports; the flags auto-clear on the next
 * tenure's ADDR write.
 *
 * ISR wiring (one vector per SERCOM, app glue as always):
 *   extern "C" void SERCOM3_Handler() {
 *       if (I2cHw::isr()) { brio::post<I2c>(brio::TransferDone{I2cHw::status()}); }
 *   }
 */
template <uint8_t n, I2cPads pads, uint8_t generator = 0>
class I2cHost {
    using S = I2cm<n>;

    static_assert(i2c_pads_valid(pads),
                  "an I2C host needs its two pins stated: SDA is PAD[0] and SCL is "
                  "PAD[1] by the chapter (33.4), and only table 6-7's pins carry I2C "
                  "at all - which no header symbol encodes, so state what you wired");

    using SdaPin = Pin<pads.sda_pin.port, pads.sda_pin.pin>;
    using SclPin = Pin<pads.scl_pin.port, pads.scl_pin.pin>;

public:
    I2cHost() = delete;

    using Resource = S;
    static constexpr I2cPads pin_pads = pads;
    static constexpr uint8_t core_generator = generator;

    struct Request {
        uint8_t addr;          ///< 7-bit client address (unshifted)
        /// Bytes written after START, LENT until the reply lands (may be
        /// null if tx_len == 0).
        Borrowed<const uint8_t, Lease::reply> tx;
        uint8_t tx_len;
        /// Where the bytes read after the (repeated) START+R land; LENT
        /// until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint8_t rx_len;
        ReplyTo<I2cDone> reply;
        I2cSpeed speed = I2cSpeed::standard_100k;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    /**
     * @brief Bring the instance up as a bus host.
     *
     * `rise_ns` is the BUS'S rise time - the pull-ups' and the wire's,
     * not the chip's - and it enters the baud arithmetic exactly as it
     * does on the AVR (a budget that ignores it lands T_LOW under the
     * specification floor). State what the bench measures; 300 ns is a
     * conservative default for a 1.5k / breadboard-scale bus.
     *
     * `core_hz` is the RATE OF THE GENERATOR the `generator` template
     * argument names - the caller's claim, exactly as freqm's
     * reference_hz is (a divided generator's rate is not knowable
     * here); 0 means generator 0 at the CPU clock, the default. WHY A
     * CALLER WOULD SLOW THE CORE AT ALL: the I2C bus monitor samples
     * SDA/SCL on this clock and HAS NO INPUT FILTER, so on a wire
     * whose crosstalk glitches are ~100 ns a fast core SEES them - as
     * false Start/Stop conditions, i.e. instant BUSERR/ARBLOST. On the
     * phase F bench (the seven-wire bundle) the measured ladder is:
     * 6 MHz core clean, 12 MHz and up dead on the first tenure, at
     * EVERY SCL rate. A clean, short, separated wire has no such
     * problem; a bundled one wants a core a notch above its top SCL
     * and no more.
     *
     * The three speeds' register pairs are resolved HERE (and at
     * rebase()); one this core cannot produce is marked unreachable -
     * speed_ok() tells, and a Request naming it completes on the spot
     * with i2c_rejected rather than running at a rate nobody asked
     * for.
     */
    template <typename Clock>
    static bool init(Clock clock, uint32_t rise_ns = 300u, uint32_t core_hz = 0u) {
        static_assert(clock_follows<Clock, I2cHost>(),
                      "this I2cHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its baud table would go stale on a "
                      "clock change");
        Nvic::disable(S::irq());
        rise_ns_ = rise_ns;
        core_override_ = core_hz;
        if (!rebase(clock_hz(clock))) {
            return false;
        }
        S::bus_clock(true);
        if (!S::core_clock(generator)) {
            return false;
        }
        if (!S::reset()) {
            return false;
        }
        applied_ = I2cSpeed::standard_100k;
        I2cmConfig c{};
        c.pads = pads;
        c.speed = applied_;
        c.baud = table_[0];
        // The BUSY-state escape: without it a glitch that looked like a
        // Start leaves the state machine BUSY forever on a two-node
        // bus. 205 us of quiet is longer than any legal Sm frame gap.
        c.inactive_timeout = I2cInactiveTimeout::us205;
        if (!S::configure(c)) {
            return false;
        }
        if (!S::enable(true)) {
            return false;
        }
        // Pads to the SERCOM only with the peripheral up (open-drain:
        // the pull-ups own the idle level, so there is no glitch to
        // dodge - the order is kept for uniformity with the siblings).
        SdaPin::function(pads.sda_pin.function, {.input_enable = true});
        SclPin::function(pads.scl_pin.function, {.input_enable = true});
        // A freshly enabled host is in UNKNOWN and must be told the bus
        // is idle (33.6.2.3); the INACTOUT above would get there too,
        // eventually, but a deliberate init does not wait on a timeout.
        if (!S::force_idle()) {
            return false;
        }
        S::enable_interrupt(I2cmFlag::all, true);
        Nvic::enable(S::irq());
        return true;
    }

    /// The core clock changed (DynamicClock fan-out): recompute the
    /// three speeds' register pairs against the CORE rate (the override
    /// when one was stated - a divided generator does not follow the
    /// CPU - and the new CPU rate otherwise). A speed the core cannot
    /// produce is marked UNREACHABLE, not an error: speed_ok() answers,
    /// and only a request naming it is refused. False only when even
    /// standard_100k is unreachable - a core that slow serves nothing.
    static bool rebase(uint32_t hz) {
        ref_hz_ = core_override_ != 0u ? core_override_ : hz;
        for (uint8_t i = 0; i < 3u; ++i) {
            const auto s = static_cast<I2cSpeed>(i);
            const auto b = i2c_baud_for(ref_hz_, i2c_speed_hz(s), rise_ns_,
                                        s == I2cSpeed::fast_plus_1m);
            valid_[i] = b.has_value();
            table_[i] = b.value_or(I2cBaud{});
        }
        return valid_[0];
    }

    /// Can this core produce that speed at all? (On a slowed core the
    /// fast end of the vocabulary drops off first.)
    static bool speed_ok(I2cSpeed s) { return valid_[static_cast<uint8_t>(s)]; }

    /// What a speed really runs at on this bus (the actual_scl
    /// readback: the divisor's truth, rise time included).
    static uint32_t scl_hz(I2cSpeed s) {
        return i2c_scl_hz(ref_hz_, table_[static_cast<uint8_t>(s)], rise_ns_);
    }
    static I2cBaud baud_of(I2cSpeed s) { return table_[static_cast<uint8_t>(s)]; }
    static uint32_t reference_hz() { return ref_hz_; }

    /// The engine is between tenures (nothing in flight). BusMaster
    /// serializes, so this is a convenience for suites, not a lock.
    static bool idle() { return phase_ == Phase::idle; }

    /**
     * @brief Begin one bus tenure (called by I2cBus from main context).
     * @return false ALWAYS on success - the tenure runs on the ISR and a
     * TransferDone{status()} follows - and true only for the degenerate
     * failure the reply must not wait for: a request whose speed cannot
     * be programmed. The avrdx TwiHost contract.
     *
     * A tenure against a BUSY bus is the silicon's to hold: writing
     * ADDR while another host owns the wire parks the START until the
     * bus goes idle (33.6.2.4.2), which is exactly the AVR's
     * held-START behaviour.
     */
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        if (!speed_ok(r.speed)) {
            // The core in force cannot produce this speed (a slowed
            // core drops the fast end of the vocabulary): refuse the
            // request on the spot rather than run at a rate nobody
            // asked for. i2c_rejected is the refused-without-moving
            // word the vocabulary already has.
            status_ = i2c_rejected;
            phase_ = Phase::idle;
            return true;
        }
        apply(r.speed);
        status_ = i2c_ok;
        // Direction of the OPENING address: write when there are bytes
        // to write or the request is a pure probe; read otherwise.
        const bool opens_read = (r.tx_len == 0u && r.rx_len != 0u);
        phase_ = opens_read ? Phase::reading : Phase::writing;
        if (!S::start_address(r.addr, opens_read)) {
            status_ = i2c_bus_error;   // SYSOP never settled: report, don't hang
            phase_ = Phase::idle;
            return true;
        }
        return false;
    }

    /// The engine's completion status, read by the app glue for the
    /// TransferDone payload.
    static uint8_t status() { return status_; }

    /**
     * @brief SERCOM interrupt body - call from SERCOMn_Handler().
     * @return true when the tenure just completed (Stop sent or bus
     * lost): the edge on which the glue posts TransferDone.
     */
    [[gnu::always_inline]] static bool isr() {
        const uint8_t p = S::pending();
        if (p == 0u) {
            return false;
        }
        if (phase_ == Phase::idle) {
            // A flag with no tenure owning it: SWEEP IT, or the level
            // holds the NVIC line and the handler storms. THE FIRST
            // VERSION RETURNED WITHOUT CLEARING and was caught doing so
            // by halt-and-dump (IPSR = this SERCOM's IRQ, INTFLAG.ERROR
            // = 0x80 standing, main starved for ever): one wire fault
            // raises MB AND ERROR together, the MB branch below
            // finished the tenure, and the leftover ERROR met exactly
            // this guard.
            S::clear_flags(I2cmFlag::all);
            S::clear_status(I2cmStatus::w1c_all);
            return false;
        }
        // MB: a byte (or the address) went out - or failed to.
        if ((p & I2cmFlag::mb) != 0u) {
            if (S::arb_lost()) {
                // The bus is no longer ours and a Stop is not ours to
                // send (33.6.2.4.2 case 1). BUSERR beside it is the
                // protocol-violation flavour of the same loss.
                return finish(S::bus_error() ? i2c_bus_error : i2c_arb_lost);
            }
            if (S::rx_nack()) {
                // Address or data NACK: the position is the truth. At
                // pos_ == 0 no data byte has moved in either phase, so
                // the unacknowledged byte was the ADDRESS - the probe's
                // answer and the nobody-home case alike; any later MB
                // with RXNACK is a refused DATA byte (a read tenure has
                // no later case - the host does that acking).
                const uint8_t st = pos_ == 0u ? i2c_nack_addr : i2c_nack_data;
                (void)S::stop();
                return finish(st);
            }
            if (phase_ == Phase::writing && pos_ < req_.tx_len) {
                (void)S::data(req_.tx.get()[pos_]);
                ++pos_;
                return false;
            }
            if (phase_ == Phase::writing && req_.rx_len != 0u) {
                // The write half is done: repeated START, read
                // direction, same address (33.6.2.4.2 case 3's second
                // option). pos_ restarts for the read half.
                phase_ = Phase::reading;
                pos_ = 0;
                (void)S::start_address(req_.addr, true);
                return false;
            }
            // Nothing left (a pure write, or the empty probe): Stop.
            (void)S::stop();
            return finish(i2c_ok);
        }
        // SB: a byte arrived (host read). Read it, then answer: ACK +
        // next (CMD 0x2) or NACK + Stop (CMD 0x3) - the acknowledge
        // action rides the command (33.10.2).
        if ((p & I2cmFlag::sb) != 0u) {
            const uint8_t v = S::data();
            if (req_.rx.get() != nullptr && pos_ < req_.rx_len) {
                req_.rx.get()[pos_] = v;
            }
            ++pos_;
            if (pos_ < req_.rx_len) {
                (void)S::read_next();
                return false;
            }
            (void)S::stop(true);   // NACK the last byte, then Stop
            return finish(i2c_ok);
        }
        // ERROR without MB: a time-out or a bus error seen from the
        // sidelines.
        if ((p & I2cmFlag::error) != 0u) {
            return finish(S::arb_lost() ? i2c_arb_lost : i2c_bus_error);
        }
        return false;
    }

    /**
     * @brief The classic bus unstick: nine SCL pulses and a Stop, by
     * hand, open-drain, with the pads reclaimed from the SERCOM for the
     * duration - avrdx/twi.hpp's Twi<n>::unstick() ported to this
     * silicon. RECOVER() FIXES THE PERIPHERAL, THIS FIXES THE WIRE.
     *
     * @return the number of pulses it took a stuck client to release
     * SDA (0 = the wire was never stuck), or 0xFF when nine pulses and
     * a Stop left SDA still low - a short, not a client.
     *
     * The bus is then re-inited from force_idle(); the caller re-inits
     * nothing.
     */
    static uint8_t unstick() {
        Nvic::disable(S::irq());
        // The pads back to PORT: open-drain by DIRECTION (OUT stays 0;
        // driving low = output, releasing = input under the bus's own
        // pull-ups), the AVR verb's exact technique.
        SdaPin::release();
        SclPin::release();
        SdaPin::clear();
        SclPin::clear();
        SdaPin::configure({.input_enable = true});
        SclPin::configure({.input_enable = true});
        // A HEALTHY WIRE IS LEFT ALONE: SDA already high means nothing
        // is stuck and zero pulses is both the answer and the action -
        // the first version pulsed first and asked after, so a clean
        // bus read "released at pulse 1" and nine spurious clocks went
        // out besides.
        if (SdaPin::read()) {
            SdaPin::function(pads.sda_pin.function, {.input_enable = true});
            SclPin::function(pads.scl_pin.function, {.input_enable = true});
            (void)S::force_idle();
            S::clear_flags(I2cmFlag::all);
            Nvic::enable(S::irq());
            return 0;
        }
        uint8_t pulses = 0;
        uint8_t released_at = 0xFF;
        for (uint8_t i = 0; i < 9u && released_at == 0xFF; ++i) {
            SclPin::output();       // SCL low
            spin_half_bit();
            SclPin::input();        // SCL released high
            spin_half_bit();
            ++pulses;
            if (SdaPin::read()) {
                released_at = pulses;
            }
        }
        // A Stop: SDA low, SCL high, SDA released while SCL is high.
        SdaPin::output();
        spin_half_bit();
        SdaPin::input();
        spin_half_bit();
        const bool free_now = SdaPin::read();
        // Hand the pads back and put the bus state machine at IDLE.
        SdaPin::function(pads.sda_pin.function, {.input_enable = true});
        SclPin::function(pads.scl_pin.function, {.input_enable = true});
        (void)S::force_idle();
        S::clear_flags(I2cmFlag::all);
        Nvic::enable(S::irq());
        if (!free_now) {
            return 0xFF;
        }
        return released_at == 0xFF ? 0u : released_at;
    }

    static void release() {
        Nvic::disable(S::irq());
        S::release();
        SdaPin::release();
        SclPin::release();
        phase_ = Phase::idle;
    }

private:
    enum class Phase : uint8_t { idle, writing, reading };

    /// ~5 us at 48 MHz: a 100 kHz half bit, timed by a counted spin
    /// because this is a recovery path that must not depend on any
    /// timer being alive.
    static void spin_half_bit() {
        for (volatile uint32_t i = 0; i < 60u; i = i + 1) {
        }
    }

    /// The one exit of every tenure: the status set, the phase idled,
    /// and EVERY flag and W1C status swept - one wire fault raises MB
    /// and ERROR together (measured), and a leftover level storms the
    /// vector. The statuses would also be auto-cleared by the next
    /// tenure's ADDR write; the flags would not.
    static bool finish(uint8_t st) {
        status_ = st;
        phase_ = Phase::idle;
        S::clear_flags(I2cmFlag::all);
        S::clear_status(I2cmStatus::w1c_all);
        return true;
    }

    /// BAUD and CTRLA.SPEED are enable-protected, so a speed change
    /// costs a disable/enable pair - cached, so a run of requests at
    /// one speed costs nothing (the spi.hpp apply() shape).
    static void apply(I2cSpeed s) {
        if (s == applied_) {
            return;
        }
        applied_ = s;
        (void)S::enable(false);
        const uint32_t a = (S::regs().SERCOM_CTRLA & ~SERCOM_I2CM_CTRLA_SPEED_Msk) |
                           SERCOM_I2CM_CTRLA_SPEED(i2c_speed_field(s));
        S::regs().SERCOM_CTRLA = a;
        const I2cBaud b = table_[static_cast<uint8_t>(s)];
        S::regs().SERCOM_BAUD =
            SERCOM_I2CM_BAUD_BAUD(b.baud) | SERCOM_I2CM_BAUD_BAUDLOW(b.baudlow);
        (void)S::enable(true);
        (void)S::force_idle();   // the enable re-boots the state machine
    }

    static inline Request req_{};
    static inline uint8_t pos_ = 0;
    static inline Phase phase_ = Phase::idle;
    static inline uint8_t status_ = i2c_ok;
    static inline I2cSpeed applied_ = I2cSpeed::standard_100k;
    static inline I2cBaud table_[3]{};
    static inline bool valid_[3]{};
    static inline uint32_t ref_hz_ = 0;
    static inline uint32_t core_override_ = 0;
    static inline uint32_t rise_ns_ = 300;
};

// =============================================================================
// The client task
// =============================================================================

/*
 * I2cClient<n, pads>
 *
 * The polled surface plus the ISR body - deliberately thin, the
 * SpiClient position: a client is a protocol and the protocol is the
 * application's. What this adds over the raw I2cs<n> is the ERRATUM
 * DISCIPLINE so an app cannot forget it:
 *
 *  - every address match sweeps the error bits 1.17.11 leaves behind
 *    (I2cs::answer_address does it);
 *  - STATUS.RXNACK IS INVALID AT THE FIRST DRDY of a tenure (erratum
 *    1.17.22, live, no register fix): first_drdy() is the software
 *    flag the workaround prescribes, armed by the AMATCH the app
 *    acknowledges through this class and consumed by its first DRDY.
 *    A transmitting client that trusted RXNACK on byte one would stop
 *    a read tenure that is actually being ACKed.
 *
 * ISR wiring, as always the app's:
 *   extern "C" void SERCOM3_Handler() { ... Peer::isr() ... }
 */
/// `generator` is the client's core GCLK generator - the same knob, for
/// the same reason, as the host's: the client's Start/Stop detectors
/// and address machinery sample the wire on this clock with no input
/// filter, so a bundled wire wants it slow (the host's init comment
/// carries the measured ladder).
template <uint8_t n, I2cPads pads, uint8_t generator = 0>
class I2cClient {
    using S = I2cs<n>;

    static_assert(i2c_pads_valid(pads),
                  "an I2C client needs its two pins stated: SDA is PAD[0] and SCL is "
                  "PAD[1] (33.4), and table 6-7's I2C-capable list is the caller's "
                  "obligation");

    using SdaPin = Pin<pads.sda_pin.port, pads.sda_pin.pin>;
    using SclPin = Pin<pads.scl_pin.port, pads.scl_pin.pin>;

public:
    I2cClient() = delete;

    using Resource = S;
    static constexpr I2cPads pin_pads = pads;
    static constexpr uint8_t core_generator = generator;

    using Config = I2csConfig;

    template <typename Clock>
    static bool init(Clock clock, const Config& cfg) {
        (void)clock;
        Nvic::disable(S::irq());
        S::bus_clock(true);
        if (!S::core_clock(core_generator)) {
            return false;
        }
        if (!S::reset()) {
            return false;
        }
        Config c = cfg;
        c.pads = pads;
        if (!S::configure(c)) {
            return false;
        }
        if (!S::enable(true)) {
            return false;
        }
        SdaPin::function(pads.sda_pin.function, {.input_enable = true});
        SclPin::function(pads.scl_pin.function, {.input_enable = true});
        first_drdy_ = false;
        Nvic::enable(S::irq());
        return true;
    }

    // ---- the protocol surface ------------------------------------------------

    /// An address match is pending (SCL held meanwhile - unlimited time).
    static bool addressed() { return (S::flags() & I2csFlag::amatch) != 0u; }
    /// The matched tenure's direction, valid at AMATCH (33.8.6).
    static bool host_reads() { return S::host_reads(); }
    static bool repeated_start() { return S::repeated_start(); }
    /// The address BYTE the match saw (DATA holds it at AMATCH): the
    /// 7-bit address in bits 7:1 - how a masked/ranged client learns
    /// WHICH address it was called by, and a general call reads 0x00.
    static uint8_t matched_byte() { return S::data(); }

    /// Answer the match. ARMS THE 1.17.22 FLAG: the next DRDY is the
    /// tenure's first, where RXNACK must not be believed.
    static void answer_address(bool ack) {
        first_drdy_ = ack;
        S::answer_address(ack);
    }

    /// A data event is pending (received byte, or the shifter wants the
    /// next transmit byte - STATUS.DIR says which).
    static bool data_ready() { return (S::flags() & I2csFlag::drdy) != 0u; }

    /// ERRATUM 1.17.22: true exactly once per tenure - at the first
    /// DRDY, where STATUS.RXNACK is invalid and must be ignored by a
    /// transmitting client. CONSUMES the flag.
    static bool first_drdy() {
        const bool f = first_drdy_;
        first_drdy_ = false;
        return f;
    }

    /// Host write: take the received byte, answering ACK (continue) or
    /// NACK.
    static uint8_t take(bool ack = true) { return S::take(ack); }
    /// Host read: hand the next byte out (releases the stretch).
    static void give(uint8_t v) { S::give(v); }
    /// After the host's closing NACK of a read tenure: complete it
    /// (CMD 0x2 - the machinery goes back to waiting for a start
    /// instead of stretching for a byte nobody wants).
    static void end_transaction(bool ack = true) { S::end_transaction(ack); }
    /// Host read, after DRDY with the byte sent: did the host NACK it
    /// (tenure over)? INVALID at the first DRDY - gate with
    /// first_drdy().
    static bool host_nacked() { return S::rx_nack(); }

    /// A Stop arrived (PREC). W1C.
    static bool stop_seen() { return (S::flags() & I2csFlag::stop) != 0u; }
    static void clear_stop() { S::clear_flags(I2csFlag::stop); }

    static bool collision() { return S::collision(); }
    static uint8_t flags() { return S::flags(); }
    static uint16_t raw_status() { return S::status(); }

    // ---- the ISR body --------------------------------------------------------

    /// The pending mask (I2csFlag::amatch | drdy | stop | error), 0 when
    /// this SERCOM was not the one asking. The app's glue decides - a
    /// client's protocol is the application's (the SpiClient position).
    [[gnu::always_inline]] static uint8_t isr() { return S::pending(); }

    static void enable_amatch_interrupt(bool on) {
        S::enable_interrupt(I2csFlag::amatch, on);
    }
    static void enable_drdy_interrupt(bool on) { S::enable_interrupt(I2csFlag::drdy, on); }
    static void enable_stop_interrupt(bool on) { S::enable_interrupt(I2csFlag::stop, on); }

    static void release() {
        Nvic::disable(S::irq());
        S::release();
        SdaPin::release();
        SclPin::release();
    }

private:
    static inline bool first_drdy_ = false;
};

// =============================================================================
// Compile-time pins of the chapter's arithmetic
// =============================================================================

// 48 MHz, 100 kHz, 300 ns of rise: total = 480 cycles, rise = 14,
// budget k = 456, symmetric 228/228 -> BAUDLOW 0 does not fit the even
// split rule (228 = 228), so BAUD times both halves.
static_assert(i2c_baud_for(48'000'000UL, 100'000UL, 300u, false).has_value());
static_assert(i2c_baud_for(48'000'000UL, 100'000UL, 300u, false)->baud == 228);
static_assert(i2c_baud_for(48'000'000UL, 100'000UL, 300u, false)->baudlow == 0);
static_assert(i2c_scl_hz(48'000'000UL, I2cBaud{228, 0}, 300u) == 100'000UL);
// 400 kHz: total = 120, rise 14, k = 96 -> 48 timing both halves.
static_assert(i2c_baud_for(48'000'000UL, 400'000UL, 300u, false)->baud == 48);
static_assert(i2c_baud_for(48'000'000UL, 400'000UL, 300u, false)->baudlow == 0);
// 1 MHz Fm+ with the 1:2 split: total = 48, rise (120 ns) = 5, k = 33
// -> low 22, high 11; the produced rate lands exactly on the megahertz.
static_assert(i2c_baud_for(48'000'000UL, 1'000'000UL, 120u, true)->baudlow == 22);
static_assert(i2c_baud_for(48'000'000UL, 1'000'000UL, 120u, true)->baud == 11);
static_assert(i2c_scl_hz(48'000'000UL, I2cBaud{11, 22}, 120u) == 1'000'000UL);
// Too fast for the budget: refused, not clamped in silence.
static_assert(!i2c_baud_for(48'000'000UL, 5'000'000UL, 300u, false).has_value());

} // namespace brio
