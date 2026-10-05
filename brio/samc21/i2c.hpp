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
 *              descriptor shape every target's I2cHost carries, buffers
 *              as Lease::reply loans, completion as the bus AO's
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
 *    with DATA, ADDR, a command, or a flag clear (33.10.6) - an
 *    unlimited-time-to-respond design;
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
 *    exists at the RESOURCE level; the engine speaks 7-bit, which is
 *    what the shared Request shape carries);
 *  - ERRATUM 1.17.11 (LIVE): the client's error status bits are NOT
 *    cleared when INTFLAG.AMATCH is cleared, against 33.8.6. The
 *    workaround is code: I2cs<n>::clear_errors() writes them by hand
 *    and the client task spends it on every address match;
 *  - ERRATUM 1.17.13 (LIVE): Quick Command with SCLSM = 1 raises a bus
 *    error on a repeated start. i2cm_config_valid() refuses the pair;
 *  - ERRATUM 1.17.16: SWRST claimed non-functional while ENABLE = 0 on
 *    every revision. NOT REPRODUCED in SPI mode on this silicon
 *    (samc21/spi.hpp); the enable-first discipline is kept here too
 *    and the I2C-mode disposition is unmeasured;
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
#include <span>
#include <type_traits>

#include "sam.h"

#include "samc21/clock.hpp"
#include "samc21/nvic.hpp"
#include "samc21/pin.hpp"
#include "samc21/sercom.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

// =============================================================================
// The knobs (33.8.1, 33.10.1..3)
// =============================================================================

/// The bus speed a tenure runs at - the three-step vocabulary every
/// target's I2cSpeed speaks, because they name the same I2C specification
/// classes. High-speed (3.4 MHz) is deliberately absent: errata 1.17.7
/// and 1.17.9 break its repeated starts with no workaround (see the
/// header comment).
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
 * ARGUMENT here and not a constant: a baud computed with a rise time the
 * bus does not have lands the SCL low time below the specification floor
 * by exactly the difference.
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
/// this rise time - the readback half of the arithmetic.
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
    /// DATA, where a DMA engine moves the bytes (33.6.4.1.2).
    static volatile void* data_address() { return &regs().SERCOM_DATA; }

    static void bus_clock(bool on) { Base::bus_clock(on); }
    static bool core_clock(uint8_t generator) { return Base::core_clock(generator); }

    // ---- synchronization (33.6.6) -------------------------------------------

    static bool sync_busy(uint32_t mask) { return (regs().SERCOM_SYNCBUSY & mask) != 0u; }
    static bool wait_sync(uint32_t mask, uint32_t spins = 0xFFFFu) {
        return clock_wait(regs().SERCOM_SYNCBUSY, mask, false, spins);
    }
    /// The one that matters per-operation: CMD, BUSSTATE, ADDR and DATA
    /// all synchronize through SYSOP while enabled. WAIT BEFORE STORING.
    /// The common case - the last operation synchronized long ago - is one
    /// load and a test inline; only a SYSOP still standing pays the
    /// bounded loop.
    [[gnu::always_inline]] static bool wait_sysop(uint32_t spins = 0xFFFFu) {
        if ((regs().SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) == 0u) {
            return true;
        }
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
    [[gnu::always_inline]] static void clear_status(uint16_t mask) {
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
    [[gnu::always_inline]] static void clear_flags(uint8_t mask) { regs().SERCOM_INTFLAG = mask; }
    [[gnu::always_inline]] static void enable_interrupt(uint8_t mask, bool on) {
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
    [[gnu::always_inline]] static bool start_address(uint8_t addr7, bool read,
                                                     uint32_t spins = 0xFFFFu) {
        if (!wait_sysop(spins)) {
            return false;
        }
        regs().SERCOM_ADDR = SERCOM_I2CM_ADDR_ADDR(
            (static_cast<uint32_t>(addr7) << 1) | (read ? 1u : 0u));
        return true;
    }

    /// The same with the AUTOMATIC LENGTH (ADDR.LENEN with ADDR.LEN,
    /// 33.10.9, 33.6.4.1.2): after `len` data bytes the host sends the
    /// STOP by itself - a read NACKs the last byte first - and a client's
    /// NACK before `len` written bytes ends the tenure with a STOP,
    /// STATUS.LENERR and INTFLAG.ERROR instead of an MB (measured on
    /// this silicon, docs/samc21/i2c.md). Smart mode must be on (33.6.4.1).
    [[gnu::always_inline]] static bool start_address_len(uint8_t addr7, bool read, uint8_t len,
                                                         uint32_t spins = 0xFFFFu) {
        if (!wait_sysop(spins)) {
            return false;
        }
        regs().SERCOM_ADDR =
            SERCOM_I2CM_ADDR_ADDR((static_cast<uint32_t>(addr7) << 1) | (read ? 1u : 0u)) |
            SERCOM_I2CM_ADDR_LENEN_Msk | SERCOM_I2CM_ADDR_LEN(len);
        return true;
    }

    /// One command strobe with its acknowledge action (33.10.2): the two
    /// can go in one store and the action precedes the command.
    [[gnu::always_inline]] static bool command(uint8_t cmd, bool nack, uint32_t spins = 0xFFFFu) {
        if (!wait_sysop(spins)) {
            return false;
        }
        regs().SERCOM_CTRLB = (regs().SERCOM_CTRLB &
                               ~(SERCOM_I2CM_CTRLB_CMD_Msk | SERCOM_I2CM_CTRLB_ACKACT_Msk)) |
                              SERCOM_I2CM_CTRLB_CMD(cmd) |
                              (nack ? SERCOM_I2CM_CTRLB_ACKACT_Msk : 0u);
        return true;
    }
    /// CTRLB.ACKACT alone, no command: what smart mode's DATA read sends
    /// next (33.10.2: neither enable-protected nor write-synchronized).
    /// The bit persists across tenures - a read's closing NACK leaves it
    /// set - so a phase that reads must put ACK back before its first
    /// byte, or smart mode NACKs that byte and the client stops serving.
    [[gnu::always_inline]] static void ack_action(bool nack) {
        regs().SERCOM_CTRLB =
            (regs().SERCOM_CTRLB & ~(SERCOM_I2CM_CTRLB_CMD_Msk | SERCOM_I2CM_CTRLB_ACKACT_Msk)) |
            (nack ? SERCOM_I2CM_CTRLB_ACKACT_Msk : 0u);
    }
    [[gnu::always_inline]] static bool stop(bool nack = false, uint32_t spins = 0xFFFFu) {
        return command(0x3u, nack, spins);
    }
    [[gnu::always_inline]] static bool read_next(uint32_t spins = 0xFFFFu) {
        return command(0x2u, false, spins);
    }

    [[gnu::always_inline]] static uint8_t data() {
        return static_cast<uint8_t>(regs().SERCOM_DATA);
    }
    [[gnu::always_inline]] static bool data(uint8_t v, uint32_t spins = 0xFFFFu) {
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
 * I2cHost<n, pads, generator, TxEngine, RxEngine>
 *
 * The transfer engine, driven by util/bus_master.hpp: a Request is ONE
 * BUS TENURE - write, read, or write-then-read joined by a repeated START
 * (the register-access idiom) - and the empty request is an address
 * probe. ALWAYS asynchronous: start() returns false and a
 * TransferDone{status()} follows from the ISR, which is the engine
 * contract every target's I2cHost keeps.
 *
 * WHAT THIS SERCOM OFFERS, AND WHAT THE ENGINE TAKES (33.6.2.4, 33.6.3.2,
 * 33.6.4.1.2, 33.10.9; the measurements are docs/samc21/i2c.md's):
 *
 *  - SMART MODE (CTRLB.SMEN), ALWAYS ON: a received byte is acknowledged
 *    by the DATA read itself - no command and no SYSOP wait per byte - so
 *    a pumped read byte is one load. On THIS silicon the DATA read also
 *    clocks the next byte in whatever ACKACT says (measured: with ACKACT =
 *    NACK the read sent the NACK AND read one more byte), so the last byte
 *    is closed the other way round: the STOP command with its NACK first,
 *    while SB still holds the clock, then DATA read - "reading the last
 *    data byte after the stop condition has been sent" (33.10.10).
 *    The DMA requires smart mode (33.6.4.1) anyway.
 *
 *  - THE BYTE PUMP, one interrupt per byte (MB after a byte went out, SB
 *    after one came in), is the engine on a build without DMA engines and
 *    for a phase shorter than dma_min_bytes on one with them.
 *
 *  - THE TWO DMA REQUESTS and the AUTOMATIC LENGTH (ADDR.LEN + LENEN) are
 *    the engined phases, with the shape the silicon was MEASURED to allow:
 *      * an engined READ of r bytes writes ADDR with LEN = r, the DMA
 *        reads DATA r times and the host NACKs the last byte and sends the
 *        STOP by itself: ONE interrupt, the receive block's completion.
 *        The STOP is still on the wire at that moment and SB stands a
 *        little longer (some hundred cycles at 100 kHz); an ADDR written
 *        while SB stands is taken as a repeated START and dies in
 *        BUSERR + ARBLOST (measured), so the NEXT start() waits for SB
 *        to fall when the last tenure ended so - bounded, and in practice
 *        already over (`tail_waits()` counts the waits that spun);
 *      * an engined WRITE of w bytes writes ADDR with LEN = w + 1. LEN is
 *        what protects the DMA from a NACK - without it the controller
 *        keeps feeding DATA past a client's NACK and the host transmits
 *        it (measured) - and the extra one is what leaves a COMPLETION
 *        EDGE: with LEN = w the host sends the STOP after the last byte
 *        and no flag rises at all (measured: no MB after the last byte,
 *        none after the automatic STOP), while with LEN = w + 1 the MB
 *        after byte w stands with the clock held, its acknowledge read,
 *        for the engine's STOP or repeated START. A NACK anywhere before
 *        is LENERR + ERROR with the STOP already sent. The block starts
 *        at the ADDRESS's MB, not before: that MB is the one proof the
 *        address was acknowledged (a NACK under LEN is the same LENERR
 *        for the address and for a data byte), and the transmit request
 *        standing then fires the first beat on the channel's enable. MB
 *        is DISARMED while the block runs - every beat raises and clears
 *        it, and an armed MB is an interrupt per byte (measured) - and
 *        re-armed by the transmit block's completion. THREE interrupts,
 *        whatever w: the address, the block, the last byte. A write of
 *        255 bytes cannot say LEN = 256 and takes the pump.
 *  - QUICK COMMAND (QCEN) is not used: the empty probe is the same
 *    tenure with no data and costs one MB either way.
 *
 * Status vocabulary (util/i2c_bus.hpp): i2c_nack_addr when an address
 * byte went unacknowledged (a write's, a read's, the repeated START's),
 * i2c_nack_data for a written data byte, i2c_arb_lost when another host
 * won the wire, i2c_bus_error for a protocol violation (and for a DMA
 * transfer error on an engined phase, the bus released). After
 * arbitration is lost the silicon has already released the bus and a
 * Stop is NOT ours to send (33.6.2.4.2 case 1); under LEN a NACK's STOP
 * is the silicon's too.
 *
 * THE DMAC BLOCK IS THE APP'S, as for every engine of this stratum:
 * Dmac::init() once before an engined init(), and DMAC_Handler bound.
 *
 * ISR wiring (one vector per SERCOM, app glue as always):
 *   extern "C" void SERCOM3_Handler() {
 *       if (I2cHw::isr()) { brio::post<I2c>(brio::TransferDone{I2cHw::status()}); }
 *   }
 *   // engine users bind the DMAC's vector as well:
 *   extern "C" void DMAC_Handler() {
 *       while (const auto irq = brio::Dmac::take_pending()) {
 *           if (I2cHw::dma_isr(irq->channel, irq->flags)) {
 *               brio::post<I2c>(brio::TransferDone{I2cHw::status()});
 *           }
 *       }
 *   }
 */
template <uint8_t n, I2cPads pads, uint8_t generator = 0,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
class I2cHost {
    using S = I2cm<n>;

    static_assert(i2c_pads_valid(pads),
                  "an I2C host needs its two pins stated: SDA is PAD[0] and SCL is "
                  "PAD[1] by the chapter (33.4), and only table 6-7's pins carry I2C "
                  "at all - which no header symbol encodes, so state what you wired");
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from samc21/dmac.hpp, or NoDmaEngine (the default)");
    static_assert(uart_engines_distinct<TxEngine, RxEngine>(),
                  "the two engines must ride two different DMA channels");

    using SdaPin = Pin<pads.sda_pin.port, pads.sda_pin.pin>;
    using SclPin = Pin<pads.scl_pin.port, pads.scl_pin.pin>;

public:
    I2cHost() = delete;

    using Resource = S;
    static constexpr I2cPads pin_pads = pads;
    static constexpr uint8_t core_generator = generator;
    /// Whether the write and the read phases can ride the DMAC - each
    /// slot on its own: a write-only device needs the transmit engine
    /// alone.
    static constexpr bool tx_engined = TxEngine::present;
    static constexpr bool rx_engined = RxEngine::present;

    /// THE SHORTEST PHASE THE ENGINES TAKE, in data bytes: below it the
    /// phase runs on the byte pump even with the engines named. Measured
    /// by bench_samc letter i at 48 MHz from flash (docs/samc21/i2c.md):
    /// a pumped byte is one entry of about 150 to 210 cycles; an engined
    /// write is three entries whatever its length and a start() some 55
    /// cycles dearer than the pump's, so it pays from the fourth byte; an
    /// engined read is one entry and a start() some 180 cycles dearer, so
    /// it pays from the third. One constant serves both within a byte.
    static constexpr uint8_t dma_min_bytes = 4;

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

    /// Bring the instance up as a bus host.
    ///
    /// `rise_ns` is the BUS'S rise time - the pull-ups' and the wire's,
    /// not the chip's - and it enters the baud arithmetic directly (a
    /// budget that ignores it lands T_LOW under the specification
    /// floor). State what the bench measures; 300 ns is a conservative
    /// default for a 1.5k / breadboard-scale bus.
    ///
    /// `core_hz` is the RATE OF THE GENERATOR the `generator` template
    /// argument names - the caller's claim, exactly as freqm's
    /// reference_hz is (a divided generator's rate is not knowable
    /// here); 0 means generator 0 at the CPU clock, the default. WHY A
    /// CALLER WOULD SLOW THE CORE AT ALL: the I2C bus monitor samples
    /// SDA/SCL on this clock and HAS NO INPUT FILTER, so on a wire
    /// whose crosstalk glitches are ~100 ns a fast core SEES them - as
    /// false Start/Stop conditions, i.e. instant BUSERR/ARBLOST. On a
    /// seven-wire bundle the measured ladder is: 6 MHz core clean,
    /// 12 MHz and up dead on the first tenure, at EVERY SCL rate. A
    /// clean, short, separated wire has no such problem; a bundled one
    /// wants a core a notch above its top SCL and no more.
    ///
    /// The three speeds' register pairs are resolved HERE (and at
    /// rebase()); one this core cannot produce is marked unreachable -
    /// speed_ok() tells, and a Request naming it completes on the spot
    /// with i2c_rejected rather than running at a rate nobody asked for.
    ///
    /// With engines named, they are armed here on the instance's DATA and
    /// its two trigger codes; the DMAC itself must already be up.
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
        if (!configure_applied(I2cSpeed::standard_100k)) {
            return false;
        }
        // Pads to the SERCOM only with the peripheral up (open-drain:
        // the pull-ups own the idle level, so there is no glitch to
        // dodge - the order is kept for uniformity with the siblings).
        SdaPin::function(pads.sda_pin.function, {.input_enable = true});
        SclPin::function(pads.scl_pin.function, {.input_enable = true});
        // A freshly enabled host is in UNKNOWN and must be told the bus
        // is idle (33.6.2.3); the INACTOUT the configuration sets would
        // get there too, eventually, but a deliberate init does not wait
        // on a timeout.
        if (!S::force_idle()) {
            return false;
        }
        arm_engines();
        phase_ = Phase::idle;
        tail_ = false;
        S::enable_interrupt(I2cmFlag::all, false);
        S::enable_interrupt(I2cmFlag::error, true);
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

    /// The resource configuration this engine runs at a speed - what a
    /// caller that reconfigures the resource behind the engine (an SMBus
    /// time-out switched on, say) starts from, so that what the engine
    /// relies on stays as it was: SMEN above all, which every read the
    /// engine pumps or engines assumes (the class comment).
    static I2cmConfig configuration(I2cSpeed s) {
        I2cmConfig c{};
        c.pads = pads;
        c.speed = s;
        c.baud = table_[static_cast<uint8_t>(s)];
        // The BUSY-state escape: without it a glitch that looked like a
        // Start leaves the state machine BUSY forever on a two-node
        // bus. 205 us of quiet is longer than any legal Sm frame gap.
        c.inactive_timeout = I2cInactiveTimeout::us205;
        c.smart = true;
        return c;
    }

    /// How many start() calls found the last engined read's SB still
    /// standing and waited for it (the class comment) - the bench's
    /// reading of whether that wait ever costs a dispatch anything.
    static uint32_t tail_waits() { return tail_waits_; }

    /// Begin one bus tenure (called by I2cBus from main context). Returns
    /// false whenever the wire moves - the tenure runs on the ISR and a
    /// TransferDone{status()} follows - and true only for the one failure
    /// that moves nothing, answered i2c_rejected through status(): a
    /// request whose speed cannot be programmed. That is the I2cHost
    /// contract (docs/design/i2c-bus.md).
    ///
    /// A tenure against a BUSY bus is the silicon's to hold: writing ADDR
    /// while another host owns the wire parks the START until the bus
    /// goes idle (33.6.2.4.2) - the held-START behaviour.
    static bool start(const Request& r) {
        // What the engine reads, field by field - the reply capsule is
        // the arbiter's, and a copy of the whole Request is a memcpy call
        // of 25 bytes on this core.
        addr_ = r.addr;
        tx_ = r.tx.get();
        tx_len_ = r.tx_len;
        rx_ = r.rx.get();
        rx_len_ = r.rx_len;
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
        if constexpr (rx_engined) {
            if (tail_) {
                // The last tenure was an engined read: its automatic NACK
                // and STOP may still hold SB (the class comment).
                tail_ = false;
                if ((S::flags() & I2cmFlag::sb) != 0u) {
                    ++tail_waits_;
                    uint32_t spins = tail_spins;
                    while ((S::flags() & I2cmFlag::sb) != 0u && --spins != 0u) {
                    }
                }
            }
        }
        apply(r.speed);
        status_ = i2c_ok;
        const bool ok = (r.tx_len == 0u && r.rx_len != 0u) ? begin_read() : begin_write();
        if (!ok) {
            status_ = i2c_bus_error;   // SYSOP never settled: report, don't hang
            phase_ = Phase::idle;
            return true;
        }
        return false;
    }

    /// The engine's completion status, read by the app glue for the
    /// TransferDone payload.
    static uint8_t status() { return status_; }

    /// SERCOM interrupt body - call from SERCOMn_Handler(). Returns true
    /// when the tenure just completed (Stop sent or bus lost): the edge
    /// on which the glue posts TransferDone.
    ///
    /// THE BYTE'S PATH IS THE BODY, THE TENURE'S IS A CALL: the two
    /// per-byte cases of the pump - a byte to read that is not the last,
    /// a byte to write after an acknowledged one - are this inline body
    /// and nothing else; every once-a-tenure decision (the last byte, the
    /// repeated START, the STOP, a NACK, a loss, an error, the engined
    /// write's block start) is settle(), one call out of line. The
    /// vector stays small - what a handler placed in SRAM pays for in
    /// RAM - and its per-byte path saves only what that path uses.
    [[gnu::always_inline]] static bool isr() {
        const uint8_t p = S::pending();
        const Phase ph = phase_;
        if (p == I2cmFlag::sb && ph == Phase::reading) {
            // Smart mode: the DATA read acknowledges the byte and clocks
            // the next one in (the class comment).
            const uint8_t pos = pos_;
            if (static_cast<uint8_t>(pos + 1u) < rx_len_) {
                const uint8_t v = S::data();
                uint8_t* const rx = rx_;
                if (rx != nullptr) {
                    rx[pos] = v;
                }
                pos_ = static_cast<uint8_t>(pos + 1u);
                return false;
            }
        } else if (p == I2cmFlag::mb && ph == Phase::writing) {
            const uint8_t pos = pos_;
            if (pos < tx_len_ &&
                (S::status() & (I2cmStatus::arb_lost | SERCOM_I2CM_STATUS_RXNACK_Msk)) == 0u) {
                (void)S::data(tx_[pos]);
                pos_ = static_cast<uint8_t>(pos + 1u);
                return false;
            }
        }
        if (p == 0u) {
            return false;
        }
        return settle(p, ph);
    }

    /// DMAC interrupt body - call from DMAC_Handler() with each
    /// take_pending() result (engine builds only; on an engineless host
    /// this compiles away). Returns true when the tenure just completed:
    /// the edge on which the glue posts TransferDone{status()}.
    ///
    /// The transmit block's end re-arms MB for the last byte's edge; the
    /// receive block's end IS the read's completion. A transfer error on
    /// either ends the tenure with i2c_bus_error and the bus released.
    [[gnu::always_inline]] static bool dma_isr(uint8_t channel, uint8_t flags) {
        // take_pending() aligns the flags to bit 0 = TERR, CHINTFLAG's own
        // layout, so the device header's mask asks without dmac.hpp here.
        const bool error = (flags & DMAC_CHINTFLAG_TERR_Msk) != 0u;
        if constexpr (tx_engined) {
            if (channel == TxEngine::channel) {
                if (phase_ != Phase::dma_write) {
                    return false;
                }
                if (error) {
                    halt_engines();
                    (void)S::stop();
                    return finish(i2c_bus_error);
                }
                (void)TxEngine::complete();
                phase_ = Phase::dma_tail;
                S::enable_interrupt(I2cmFlag::mb, true);
                return false;
            }
        }
        if constexpr (rx_engined) {
            if (channel == RxEngine::channel) {
                if (phase_ != Phase::dma_read) {
                    return false;
                }
                if (error) {
                    halt_engines();
                    (void)S::stop();
                    return finish(i2c_bus_error);
                }
                (void)RxEngine::complete();
                // The silicon is sending the NACK and the STOP; INTFLAG
                // is left to it (an SB swept here and an ADDR written at
                // once is the measured failure), and the next start()
                // waits for SB to fall.
                tail_ = true;
                status_ = i2c_ok;
                phase_ = Phase::idle;
                return true;
            }
        }
        (void)channel;
        (void)error;
        return false;
    }

    /// The classic bus unstick: nine SCL pulses and a Stop, by hand,
    /// open-drain, with the pads reclaimed from the SERCOM for the
    /// duration. RECOVER() FIXES THE PERIPHERAL, THIS FIXES THE WIRE.
    ///
    /// Returns the number of pulses it took a stuck client to release SDA
    /// (0 = the wire was never stuck), or 0xFF when nine pulses and a
    /// Stop left SDA still low - a short, not a client.
    ///
    /// The bus is then re-inited from force_idle(); the caller re-inits
    /// nothing.
    static uint8_t unstick() {
        Nvic::disable(S::irq());
        // The pads back to PORT: open-drain by DIRECTION (OUT stays 0;
        // driving low = output, releasing = input under the bus's own
        // pull-ups).
        SdaPin::release();
        SclPin::release();
        SdaPin::clear();
        SclPin::clear();
        SdaPin::configure({.input_enable = true});
        SclPin::configure({.input_enable = true});
        // A HEALTHY WIRE IS LEFT ALONE: SDA already high means nothing
        // is stuck, and zero pulses is both the answer and the action.
        // Pulsing first and asking after would report a clean bus as
        // "released at pulse 1" and put nine spurious clocks on it.
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

    /// Put the ENGINE back where start() is legal: the init() tail re-run
    /// from the cached configuration, and the DMA engines put away and
    /// re-claimed. Clocks and pads are untouched - a SERCOM software reset
    /// reaches neither GCLK routing nor PORT - and the baud table stands,
    /// so no Clock is needed.
    ///
    /// The verb a timed I2cBus calls on a tenure that never answered
    /// (util/bus_master.hpp): a START parked into a held wire does NOT
    /// fire on this silicon when the hold releases (measured), so a
    /// re-init is the ONLY way out of a park. THE WIRE IS NOT ITS JOB: a
    /// client still holding SDA makes the next tenure report - a bus
    /// error, another timeout - and unstick() is the wire's verb.
    ///
    /// Returns false when a synchronization never settled (the bounded
    /// waits' honesty: a false engine is refusing, not hanging).
    static bool recover() {
        Nvic::disable(S::irq());
        phase_ = Phase::idle;
        tail_ = false;
        if constexpr (tx_engined) {
            TxEngine::stop();
        }
        if constexpr (rx_engined) {
            RxEngine::stop();
        }
        bool ok = S::reset();
        ok = configure_applied(I2cSpeed::standard_100k) && ok;
        ok = S::force_idle() && ok;
        arm_engines();
        S::enable_interrupt(I2cmFlag::all, false);
        S::enable_interrupt(I2cmFlag::error, true);
        Nvic::enable(S::irq());
        return ok;
    }

    static void release() {
        Nvic::disable(S::irq());
        if constexpr (tx_engined) {
            TxEngine::stop();
        }
        if constexpr (rx_engined) {
            RxEngine::stop();
        }
        S::release();
        SdaPin::release();
        SclPin::release();
        phase_ = Phase::idle;
        tail_ = false;
    }

private:
    /// What the tenure in flight is waiting for (the class comment):
    /// the pumped phases, the engined write's three moments, the engined
    /// read.
    enum class Phase : uint8_t {
        idle,
        writing,     ///< pump: MB after the address or a byte
        reading,     ///< pump: SB after a byte (MB only for a NACK or a loss)
        dma_addr,    ///< engined write: the address's MB starts the block
        dma_write,   ///< engined write: the block runs, MB disarmed
        dma_tail,    ///< engined write: block done, MB armed for the last byte's edge
        dma_read,    ///< engined read: the block runs, LEN closes the tenure
    };

    /// Spins of the tail wait in start(): a few thousand loads cover the
    /// longest SB tail measured (some hundred cycles at 100 kHz) many
    /// times over; running out is not an error - the wait is a courtesy
    /// to the wire, never a lock.
    static constexpr uint32_t tail_spins = 4096u;

    /// ~5 us at 48 MHz: a 100 kHz half bit, timed by a counted spin
    /// because this is a recovery path that must not depend on any
    /// timer being alive.
    static void spin_half_bit() {
        for (volatile uint32_t i = 0; i < 60u; i = i + 1) {
        }
    }

    /// Everything isr()'s inline body does not take: the last byte of a
    /// read, the end of a write phase, a NACK, a loss, an error, the
    /// engined write's block start, and a flag with no tenure. Once a
    /// tenure, so one call.
    [[gnu::noinline]] static bool settle(uint8_t p, Phase ph) {
        if (ph == Phase::idle) {
            // A flag with no tenure owning it: SWEEP IT, or the level
            // holds the NVIC line and the handler storms. One wire fault
            // raises MB AND ERROR together (measured), and the tenure's
            // own exit already took the MB.
            S::clear_flags(I2cmFlag::all);
            S::clear_status(I2cmStatus::w1c_all);
            return false;
        }
        // SB: the pumped read's LAST byte (isr() took the others). The
        // STOP command with its NACK goes FIRST, synchronized, then the
        // read (the class comment: on this silicon a smart DATA read
        // clocks one more byte in whatever ACKACT says).
        if ((p & I2cmFlag::sb) != 0u) {
            const uint8_t pos = pos_;
            (void)S::stop(true);
            (void)S::wait_sysop();
            const uint8_t v = S::data();
            if (rx_ != nullptr && pos < rx_len_) {
                rx_[pos] = v;
            }
            return finish(i2c_ok);
        }
        // MB: a byte (or an address) went out - or failed to.
        if ((p & I2cmFlag::mb) != 0u) {
            const uint16_t st = S::status();   // one snapshot: one crossing, one truth
            if ((st & I2cmStatus::arb_lost) != 0u) {
                // The bus is no longer ours and a Stop is not ours to
                // send (33.6.2.4.2 case 1). BUSERR beside it is the
                // protocol-violation flavour of the same loss.
                return finish((st & I2cmStatus::bus_error) != 0u ? i2c_bus_error : i2c_arb_lost);
            }
            if ((st & SERCOM_I2CM_STATUS_RXNACK_Msk) != 0u) {
                // A pumped phase's NACK (under LEN a NACK is LENERR on
                // ERROR, below): at pos_ == 0 nothing of the phase moved,
                // so the unacknowledged byte was its ADDRESS.
                (void)S::stop();
                return finish(pos_ == 0u ? i2c_nack_addr : i2c_nack_data);
            }
            if (ph == Phase::writing) {
                const uint8_t pos = pos_;
                if (pos < tx_len_) {          // (an MB beside an ERROR: still a byte to go)
                    (void)S::data(tx_[pos]);
                    pos_ = static_cast<uint8_t>(pos + 1u);
                    return false;
                }
            } else if (ph == Phase::dma_addr) {
                // The address was acknowledged: the block starts now, on
                // the transmit request this MB stands beside, and MB is
                // disarmed until the block's completion (the class
                // comment).
                if constexpr (tx_engined) {
                    S::enable_interrupt(I2cmFlag::mb, false);
                    phase_ = Phase::dma_write;
                    if (!TxEngine::start(std::span<const uint8_t>(tx_, tx_len_))) {
                        (void)S::stop();
                        return finish(i2c_bus_error);
                    }
                    return false;
                }
            } else if (ph != Phase::dma_tail) {
                (void)S::stop();             // an MB no phase expects
                return finish(i2c_bus_error);
            }
            // The write phase is complete (the pump's last byte, or the
            // block's, acknowledged): a repeated START into the read
            // phase, or the STOP.
            if (rx_len_ != 0u) {
                if (!begin_read()) {
                    (void)S::stop();
                    return finish(i2c_bus_error);
                }
                return false;
            }
            (void)S::stop();
            return finish(i2c_ok);
        }
        // ERROR alone: under LEN a NACK (LENERR, the STOP already sent by
        // the silicon); otherwise a lost bus, a bus error or a time-out
        // seen from the sidelines.
        const uint16_t st = S::status();
        halt_engines();
        if ((st & I2cmStatus::len_error) != 0u) {
            return finish((ph == Phase::dma_write || ph == Phase::dma_tail) ? i2c_nack_data
                                                                            : i2c_nack_addr);
        }
        return finish((st & I2cmStatus::arb_lost) != 0u ? i2c_arb_lost : i2c_bus_error);
    }

    /// The write phase: the address with W, then the pump or the block.
    [[gnu::always_inline]] static bool begin_write() {
        pos_ = 0;
        if constexpr (tx_engined) {
            if (tx_len_ >= dma_min_bytes && tx_len_ < 255u) {
                phase_ = Phase::dma_addr;
                S::enable_interrupt(I2cmFlag::mb | I2cmFlag::error, true);
                return S::start_address_len(addr_, false,
                                            static_cast<uint8_t>(tx_len_ + 1u));
            }
        }
        phase_ = Phase::writing;
        S::enable_interrupt(I2cmFlag::mb | I2cmFlag::error, true);
        return S::start_address(addr_, false);
    }

    /// The read phase: the address with R - the opening one, or the
    /// repeated START from a write phase's last MB - then the pump or the
    /// block (its engine started BEFORE the address, so the first byte's
    /// request finds it running).
    [[gnu::always_inline]] static bool begin_read() {
        pos_ = 0;
        S::ack_action(false);   // the last read's closing NACK still stands in ACKACT
        if constexpr (rx_engined) {
            uint8_t* const rx = rx_;
            if (rx_len_ >= dma_min_bytes && rx != nullptr) {
                phase_ = Phase::dma_read;
                S::enable_interrupt(I2cmFlag::mb | I2cmFlag::sb, false);
                S::enable_interrupt(I2cmFlag::error, true);
                if (!RxEngine::start(std::span<uint8_t>(rx, rx_len_))) {
                    return false;
                }
                return S::start_address_len(addr_, true, rx_len_);
            }
        }
        phase_ = Phase::reading;
        S::enable_interrupt(I2cmFlag::all, true);
        return S::start_address(addr_, true);
    }

    /// End whatever block is in flight, keeping the bindings.
    [[gnu::always_inline]] static void halt_engines() {
        if constexpr (tx_engined) {
            if (phase_ == Phase::dma_write) {
                TxEngine::halt();
            }
        }
        if constexpr (rx_engined) {
            if (phase_ == Phase::dma_read) {
                RxEngine::halt();
            }
        }
    }

    /// The one exit of every pumped or failed tenure: the status set,
    /// the phase idled, MB and SB disarmed (ERROR stays: the idle guard
    /// sweeps it; every phase arms it again with its own flags, in the same
    /// store, so a resource reconfigured behind the engine - configure()
    /// disarms everything - cannot leave a tenure deaf to its errors), and
    /// EVERY flag and W1C status swept - one wire fault
    /// raises MB and ERROR together (measured), and a leftover level
    /// storms the vector. The statuses would also be auto-cleared by the
    /// next tenure's ADDR write; the flags would not.
    [[gnu::always_inline]] static bool finish(uint8_t st) {
        status_ = st;
        phase_ = Phase::idle;
        S::enable_interrupt(I2cmFlag::mb | I2cmFlag::sb, false);
        S::clear_flags(I2cmFlag::all);
        S::clear_status(I2cmStatus::w1c_all);
        return true;
    }

    /// The whole configuration at one speed, written disabled (CTRLA,
    /// CTRLB with SMEN, BAUD are enable-protected), then enabled.
    static bool configure_applied(I2cSpeed s) {
        applied_ = s;
        return S::configure(configuration(s)) && S::enable(true);
    }

    static void arm_engines() {
        if constexpr (tx_engined) {
            TxEngine::arm(S::data_address(), S::dma_tx_trigger());
        }
        if constexpr (rx_engined) {
            RxEngine::arm(S::data_address(), S::dma_rx_trigger());
        }
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

    static inline const uint8_t* tx_ = nullptr;
    static inline uint8_t* rx_ = nullptr;
    static inline uint8_t addr_ = 0;
    static inline uint8_t tx_len_ = 0;
    static inline uint8_t rx_len_ = 0;
    static inline uint8_t pos_ = 0;
    static inline Phase phase_ = Phase::idle;
    static inline uint8_t status_ = i2c_ok;
    static inline bool tail_ = false;
    static inline uint32_t tail_waits_ = 0;
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
