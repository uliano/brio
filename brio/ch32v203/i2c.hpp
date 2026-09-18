/*
 * i2c.hpp
 *
 * The I2C of the CH32V203 (RM ch. 19): up to two instances, host and
 * client, 7- and 10-bit addresses, dual addressing and the general
 * call, two speeds with fast mode's two duty shapes, clock stretching,
 * PEC and the SMBus bits, two DMA requests per instance - the STM32F1's
 * I2C under WCH's register names, an EVENT MACHINE (SB, ADDR, BTF,
 * ADD10, STOPF, RxNE, TxE and the errors) over two vectors per
 * instance, one for events and one for errors. Three layers, the other
 * strata's arrangement:
 *
 *  - `I2c<n>` is the RESOURCE: the register block, its gate and reset,
 *    the clock arithmetic, the flags and the sequences that clear them,
 *    the control bits as verbs. It decides nothing.
 *  - `I2cHost<n, pins, TxEngine, RxEngine>` is the ENGINE util/
 *    i2c_bus.hpp's I2cBus drives: the Request is the other strata's
 *    VERBATIM - {addr, tx span, rx span, reply, speed}, one bus tenure
 *    that is a write, a read, a write-then-read with a repeated START,
 *    or the empty probe - and the outcome vocabulary is i2c_bus.hpp's
 *    (a NACK on the address, a NACK on a byte, arbitration lost, a bus
 *    error).
 *  - `I2cClient<n, pins>` is the target side: a polled surface and two
 *    ISR bodies, deliberately thin, because a client is a protocol and
 *    which protocol is the application's.
 *
 * THE MACHINE THIS CHAPTER PRESCRIBES is a set of software sequences,
 * each stretching SCL low until it completes (19.3's notes): EVT5 (SB
 * up: read STAR1, write the address), EVT6 (ADDR up: read STAR1 then
 * STAR2), EVT8 (TxE: write DATAR), EVT8_2 (TxE and BTF: the last byte
 * is out, STOP or repeated START), EVT7 (RxNE: read DATAR) - and a
 * RECEIVE PROCEDURE THAT DEPENDS ON THE COUNT: one byte wants ACK
 * cleared BEFORE ADDR is cleared and STOP set right after; two bytes
 * want POS set with ACK, ACK cleared after ADDR, and the pair read
 * after one BTF; three or more run on RxNE until three remain, then
 * wait for BTF twice - the first clears ACK and reads N-2, the second
 * sets STOP and reads N-1 and N. All of it is in `isr()`, phase by
 * phase, and the reason the ITBUFEN enable is switched on and off
 * during a tenure: TxE and RxNE interrupt only while the byte pump
 * needs them, BTF (under ITEVTEN alone) carries the rest.
 *
 * THE BUS CLOCK IS PB1 AND THE CHAPTER PUTS A CEILING ON IT. CTLR2's
 * FREQ states that clock in whole megahertz and 19.12.2 confines the
 * field to 000100b..111100b, 4 to 60 MHz: the number has to FIT, so a
 * PB1 above 60 MHz has no legal FREQ and init() refuses it. The
 * stratum's PB1 ceiling is 72 MHz (clock.hpp), so this is the one
 * chapter whose peripheral cannot be used at the top of the tree - a
 * program that wants I2C keeps PB1 at or below 60 MHz, which is an
 * HCLK of 120 MHz or less through the /2 prescaler. The rise-time
 * register says the same thing from the other side: standard mode's
 * TRISE is (1000 ns x Fpclk) + 1 and its six bits overflow just past
 * 60 MHz.
 *
 * THERE IS A RISE-TIME REGISTER HERE, unlike on the CH32V00x: RTR
 * (19.12.9), six bits, reset 0x02, writable only with PE clear, holding
 * the longest rise the block must plan for as (rise / Tpclk) + 1. The
 * SCL timing is therefore three registers - CTLR2.FREQ, CKCFGR (CCR
 * under F/S and DUTY) and RTR - and `i2c_timing_for()` solves all
 * three, taking the WIRE's rise time in nanoseconds as an argument
 * because the wire is not a property of the silicon: a bench pulled up
 * by a peer's 40 kOhm internal pulls rises in about a microsecond where
 * 4.7 kOhm resistors take a tenth of that, and the block's data setup
 * and hold times are generated from what it has been told. Told
 * nothing, it is told the mode's own maximum - 1000 ns in standard
 * mode, 300 ns in fast mode, the two numbers the I2C specification
 * states.
 *
 * The CCR arithmetic is the F1's: standard mode divides by 2 x CCR,
 * fast mode by 3 x CCR at DUTY 2 and 25 x CCR at DUTY 16/9, rounded UP
 * here so a bus never runs faster than asked.
 *
 * THE DMA REQUESTS ARE CHANNELS (dma_engine.hpp, table 11-5): I2C1
 * transmits on 6 and receives on 7, I2C2 on 4 and 5. On this family the
 * channel IS the request, so an engine slot naming any other channel is
 * refused at compile time. With engines a write phase of any length and
 * a read phase of two bytes or more run on them (CTLR2.LAST makes the
 * controller NACK the last byte a receive block takes, 19.12.2); a
 * one-byte read stays on the pump, whose ACK-before-ADDR sequence the
 * DMA path cannot express. IN SLEEP THE BUS MATRIX SERVES THE CORE
 * ALONE (this stratum's finding, docs/ch32v203/README.md): a transport
 * with engines holds the program awake, exactly as usart.hpp's does.
 *
 * BUSY IS THE WIRE, AND IT CAN BE LEFT STANDING. 19.12.7 defines BUSY as
 * "SDA or SCL has a low level", cleared when a STOP is detected, so a
 * START set while the bus is busy is held by the hardware until it frees
 * - which is how two controllers' STARTs meet and the arbitration
 * decides - and a tenure that ends without the peripheral seeing its own
 * STOP leaves BUSY standing over an idle wire. That is 19.12.1's own
 * case for SWRST ("when no stop condition is detected on the bus but the
 * busy bit is 1"), it is real here after a DMA-served read (measured),
 * and `start()` answers it: when the bounded wait runs out AND BOTH
 * LINES READ HIGH the chapter's reset is taken and the whole
 * configuration written again, the wire being the guard that keeps
 * another master's busy bus out of it.
 *
 * THE PADS ARE A COLUMN (afio.hpp, table 10-34). I2C1 has two: the
 * default PB6/PB7 and code 1's PB8/PB9. I2C2 has no remap field - one
 * column, PB10/PB11, from the datasheet's pin table. `I2cPins` carries
 * the code and init() writes it, refused where the part bonds no pad of
 * it. THE SMBus ALERT PAD does not move with the column (PB5 for I2C1,
 * PB12 for I2C2, datasheet 3.2) and is the application's to claim: no
 * verb here drives it.
 *
 * HOW MANY INSTANCES a part has is `device::i2c_count` and nothing else
 * (datasheet table 2-1): the CH32V203F6 has none at all - I2C1 lives on
 * PB6/PB7 and that package bonds neither - the parts up to the
 * CH32V203K8 have I2C1 alone, and the CH32V203C8 and RB have both. An
 * instance a part has not got does not compile.
 *
 * NOT COVERED YET: 10-bit addressing on the HOST side (the resource has
 * the mode and the client matches such an address; the host's header
 * sequence, EVT9, has no user - i2c_bus.hpp's Request carries a 7-bit
 * address on every stratum); PEC as a tenure shape (the resource has
 * the enable and the register, no Request asks for it); the SMBus
 * protocols above the bits (ARP, the host notify, the alert response) -
 * there is no SMBus device on this bench; a wake from a sleep on an
 * address match, which this family's PWR chapter does not offer.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "ch32v203/afio.hpp"
#include "ch32v203/clock.hpp"
#include "ch32v203/delay.hpp"
#include "ch32v203/device.hpp"
#include "ch32v203/dma_engine.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

// =============================================================================
// The registers (tables 19-1 and 19-2): sixteen bits at a four-byte stride
// =============================================================================

struct I2cRegs {
    volatile uint16_t CTLR1;    ///< 0x00
    uint16_t RESERVED0;
    volatile uint16_t CTLR2;    ///< 0x04
    uint16_t RESERVED1;
    volatile uint16_t OADDR1;   ///< 0x08
    uint16_t RESERVED2;
    volatile uint16_t OADDR2;   ///< 0x0c
    uint16_t RESERVED3;
    volatile uint16_t DATAR;    ///< 0x10
    uint16_t RESERVED4;
    volatile uint16_t STAR1;    ///< 0x14
    uint16_t RESERVED5;
    volatile uint16_t STAR2;    ///< 0x18
    uint16_t RESERVED6;
    volatile uint16_t CKCFGR;   ///< 0x1c
    uint16_t RESERVED7;
    volatile uint16_t RTR;      ///< 0x20 the rise time
    uint16_t RESERVED8;
};

constexpr uint32_t i2c_base_for(uint8_t n) {
    return n == 1 ? pb1_base + 0x5400 : n == 2 ? pb1_base + 0x5800 : 0;
}

/// Both instances hang on PB1, which is their gate, their reset line and
/// the clock CTLR2.FREQ must state.
constexpr Bus i2c_bus_for(uint8_t) { return Bus::pb1; }

constexpr uint32_t i2c_gate_for(uint8_t n) {
    return n == 1 ? rcc_pb1_i2c1 : n == 2 ? rcc_pb1_i2c2 : 0;
}

constexpr Irq i2c_event_irq_for(uint8_t n) { return n == 1 ? Irq::i2c1_ev : Irq::i2c2_ev; }
constexpr Irq i2c_error_irq_for(uint8_t n) { return n == 1 ? Irq::i2c1_er : Irq::i2c2_er; }

/// Does THIS PART have that instance (datasheet table 2-1)?
constexpr bool i2c_present(uint8_t n) { return n >= 1u && n <= device::i2c_count; }

/// Table 11-5's two rows per instance, read through dma_engine.hpp so
/// that the channel numbers live in exactly one place.
constexpr uint8_t i2c_dma_tx_channel(uint8_t n) {
    return dma_request_channel(n == 1 ? DmaRequest::i2c1_tx : DmaRequest::i2c2_tx);
}
constexpr uint8_t i2c_dma_rx_channel(uint8_t n) {
    return dma_request_channel(n == 1 ? DmaRequest::i2c1_rx : DmaRequest::i2c2_rx);
}

// CTLR1 (19.12.1)
inline constexpr uint16_t i2c_pe        = 1u << 0;
inline constexpr uint16_t i2c_smbus     = 1u << 1;
inline constexpr uint16_t i2c_smbtype   = 1u << 3;
inline constexpr uint16_t i2c_enarp     = 1u << 4;
inline constexpr uint16_t i2c_enpec     = 1u << 5;
inline constexpr uint16_t i2c_engc      = 1u << 6;
inline constexpr uint16_t i2c_nostretch = 1u << 7;
inline constexpr uint16_t i2c_start     = 1u << 8;
inline constexpr uint16_t i2c_stop      = 1u << 9;
inline constexpr uint16_t i2c_ack       = 1u << 10;
inline constexpr uint16_t i2c_pos       = 1u << 11;
inline constexpr uint16_t i2c_pec       = 1u << 12;
inline constexpr uint16_t i2c_alert     = 1u << 13;
inline constexpr uint16_t i2c_swrst     = 1u << 15;
// CTLR2 (19.12.2)
inline constexpr uint16_t i2c_freq_mask = 0x3Fu;
inline constexpr uint16_t i2c_iterren   = 1u << 8;
inline constexpr uint16_t i2c_itevten   = 1u << 9;
inline constexpr uint16_t i2c_itbufen   = 1u << 10;
inline constexpr uint16_t i2c_dmaen     = 1u << 11;
inline constexpr uint16_t i2c_last      = 1u << 12;
// OADDR1 (19.12.3). The F1 lineage's "bit 14 must be kept at one" is NOT
// in this register description: 14:10 are reserved here.
inline constexpr uint16_t i2c_addmode   = 1u << 15;
// OADDR2 (19.12.4)
inline constexpr uint16_t i2c_endual    = 1u << 0;
// STAR1 (19.12.6)
inline constexpr uint16_t i2c_sb        = 1u << 0;
inline constexpr uint16_t i2c_addr      = 1u << 1;
inline constexpr uint16_t i2c_btf       = 1u << 2;
inline constexpr uint16_t i2c_add10     = 1u << 3;
inline constexpr uint16_t i2c_stopf     = 1u << 4;
inline constexpr uint16_t i2c_rxne      = 1u << 6;
inline constexpr uint16_t i2c_txe       = 1u << 7;
inline constexpr uint16_t i2c_berr      = 1u << 8;
inline constexpr uint16_t i2c_arlo      = 1u << 9;
inline constexpr uint16_t i2c_af        = 1u << 10;
inline constexpr uint16_t i2c_ovr       = 1u << 11;
inline constexpr uint16_t i2c_pecerr    = 1u << 12;
/// SMBus's 25 ms clock-low flag. NOT util/i2c_bus.hpp's `i2c_timeout`,
/// which is the ARBITER's status code: this one is a bit in STAR1.
inline constexpr uint16_t i2c_smbus_timeout = 1u << 14;
inline constexpr uint16_t i2c_smbalert  = 1u << 15;
/// The five an I2C tenure can meet.
inline constexpr uint16_t i2c_errors    = i2c_berr | i2c_arlo | i2c_af | i2c_ovr | i2c_pecerr;
/// The two more that only SMBus mode raises (19.12.6): the 25 ms clock
/// low and the alert pin. ITERREN routes them to the same vector.
inline constexpr uint16_t i2c_smbus_errors = i2c_smbus_timeout | i2c_smbalert;
inline constexpr uint16_t i2c_all_errors = i2c_errors | i2c_smbus_errors;
// STAR2 (19.12.7)
inline constexpr uint16_t i2c_msl        = 1u << 0;
inline constexpr uint16_t i2c_busy       = 1u << 1;
inline constexpr uint16_t i2c_tra        = 1u << 2;
inline constexpr uint16_t i2c_gencall    = 1u << 4;
inline constexpr uint16_t i2c_smbdefault = 1u << 5;
inline constexpr uint16_t i2c_smbhost    = 1u << 6;
inline constexpr uint16_t i2c_dualf      = 1u << 7;
inline constexpr uint16_t i2c_pec_shift  = 8;
// CKCFGR (19.12.8)
inline constexpr uint16_t i2c_ccr_mask  = 0x0FFFu;
inline constexpr uint16_t i2c_duty      = 1u << 14;
inline constexpr uint16_t i2c_fs        = 1u << 15;
// RTR (19.12.9)
inline constexpr uint16_t i2c_trise_mask = 0x3Fu;

// =============================================================================
// The vocabulary
// =============================================================================

/// The two speeds 19.1 names.
enum class I2cSpeed : uint8_t { standard_100k = 0, fast_400k = 1 };

inline constexpr uint8_t i2c_speed_count = 2;

constexpr uint32_t i2c_speed_hz(I2cSpeed s) {
    return s == I2cSpeed::fast_400k ? 400'000UL : 100'000UL;
}

/// What the I2C specification allows SCL to rise in, per mode - what the
/// block is told when the caller does not measure the wire.
constexpr uint16_t i2c_default_rise_ns(I2cSpeed s) {
    return s == I2cSpeed::fast_400k ? 300u : 1000u;
}

/// Fast mode's two duty shapes (CKCFGR.DUTY): Tlow/Thigh = 2, or 16/9.
enum class I2cDuty : uint8_t { ratio_2 = 0, ratio_16_9 = 1 };

/// What the three timing registers hold for one speed at one bus clock.
struct I2cTiming {
    uint8_t freq_mhz = 0;    ///< CTLR2.FREQ, the bus clock in whole MHz
    uint16_t ckcfgr = 0;     ///< CCR with F/S and DUTY
    uint8_t trise = 0;       ///< RTR, (rise / Tpclk) + 1
};

/// 19.12.2's window for FREQ: the field is six bits and the chapter
/// confines it further, to 4..60 MHz.
inline constexpr uint32_t i2c_min_hz = 4'000'000UL;
inline constexpr uint32_t i2c_max_hz = 60'000'000UL;

/**
 * The chapter's arithmetic: standard mode SCL = pclk / (2 x CCR), fast
 * mode pclk / (3 x CCR) at DUTY 2 and pclk / (25 x CCR) at DUTY 16/9,
 * CCR rounded UP - a bus at most as fast as asked - with the floors the
 * vendor's own init keeps (4 in standard mode, 1 in fast). RTR is
 * (rise_ns x Fpclk) + 1 truncated, the register's own example
 * (19.12.9), and `rise_ns` = 0 means the mode's maximum.
 *
 * Nullopt outside FREQ's window, when CCR would not fit its twelve bits,
 * or when RTR would not fit its six.
 */
constexpr std::optional<I2cTiming> i2c_timing_for(uint32_t pclk, I2cSpeed speed,
                                                  I2cDuty duty = I2cDuty::ratio_2,
                                                  uint16_t rise_ns = 0) {
    if (pclk < i2c_min_hz || pclk > i2c_max_hz) {
        return {};
    }
    const uint32_t hz = i2c_speed_hz(speed);
    uint32_t ccr = 0;
    uint16_t bits = 0;
    if (speed == I2cSpeed::standard_100k) {
        const uint32_t div = 2UL * hz;
        ccr = (pclk + div - 1UL) / div;
        if (ccr < 4UL) {
            ccr = 4UL;
        }
    } else {
        const uint32_t div = (duty == I2cDuty::ratio_16_9 ? 25UL : 3UL) * hz;
        ccr = (pclk + div - 1UL) / div;
        if (ccr < 1UL) {
            ccr = 1UL;
        }
        bits = static_cast<uint16_t>(i2c_fs | (duty == I2cDuty::ratio_16_9 ? i2c_duty : 0u));
    }
    if (ccr > i2c_ccr_mask) {
        return {};
    }
    const uint32_t rise = rise_ns != 0u ? rise_ns : i2c_default_rise_ns(speed);
    const uint32_t trise = (rise * (pclk / 1'000'000UL)) / 1000UL + 1UL;
    if (trise > i2c_trise_mask) {
        return {};
    }
    return I2cTiming{static_cast<uint8_t>(pclk / 1'000'000UL),
                     static_cast<uint16_t>(bits | static_cast<uint16_t>(ccr)),
                     static_cast<uint8_t>(trise)};
}

/// What a timing really produces, for the record.
constexpr uint32_t i2c_scl_hz(uint32_t pclk, I2cTiming t) {
    const uint32_t ccr = t.ckcfgr & i2c_ccr_mask;
    if (ccr == 0u) {
        return 0;
    }
    if ((t.ckcfgr & i2c_fs) == 0u) {
        return pclk / (2UL * ccr);
    }
    return pclk / (((t.ckcfgr & i2c_duty) != 0u ? 25UL : 3UL) * ccr);
}

/// The rate of the bus the instances hang on, out of a static or a
/// dynamic clock - the one number FREQ states and CCR divides.
template <typename C>
constexpr uint32_t i2c_bus_hz(C clock) {
    if constexpr (C::is_static) {
        (void)clock;
        return C::pclk1_hz;
    } else {
        (void)clock;
        return C::pclk1_hz();
    }
}

/// The same answer from an HCLK, which is what a dynamic clock hands a
/// user it is about to rebase (the prescalers still hold the rate being
/// left, so this is arithmetic and not a read).
constexpr uint32_t i2c_bus_hz_at(uint32_t hclk) { return pclk1_hz_at(hclk); }

/// Which pads carry the two lines, and the REMAP CODE that puts the
/// peripheral on them (afio.hpp, table 10-34): init() writes the code.
/// The SMBus alert pad is not here - it does not move with the column
/// and no verb of this file drives it.
struct I2cPins {
    Pad scl{};
    Pad sda{};
    uint8_t remap = 0;
};

/// How many columns an instance has: I2C1's two, and I2C2's one (no
/// remap field exists for it).
constexpr uint8_t i2c_column_count(uint8_t n) { return n == 1 ? afio_i2c1_codes : 1u; }

/// The pads of column `code` on that instance.
constexpr I2cPins i2c_pins_for(uint8_t n, uint8_t code) {
    const I2cPadSet p = n == 1 ? afio_i2c1_pads(code) : afio_i2c2_pad_set;
    return I2cPins{.scl = p.scl, .sda = p.sda,
                   .remap = static_cast<uint8_t>(n == 1 ? code : 0u)};
}

/// The default for `I2cHost<n>` and `I2cClient<n>`: each instance's own
/// first column.
template <uint8_t n>
inline constexpr I2cPins i2c_default_pins = i2c_pins_for(n, 0);

/**
 * Is that column usable on this part, for that instance? A bus needs
 * both lines, they must be distinct, the code must be a column the
 * instance has, and both pads must be ones the package brings out
 * (afio.hpp's tables are the series', the bonding is the part's).
 */
constexpr bool i2c_pins_valid(uint8_t n, const I2cPins& p) {
    return p.scl.valid() && p.sda.valid() && !(p.scl == p.sda) &&
           p.remap < i2c_column_count(n) && pad_bonded(p.scl) && pad_bonded(p.sda);
}

/// A target's addresses (19.4): one 7- or 10-bit own address, an
/// optional second 7-bit one (ENDUAL), the general call.
struct I2cAddressConfig {
    uint16_t own = 0;            ///< 7 bits, or 10 with ten_bit
    bool ten_bit = false;
    std::optional<uint8_t> second{};   ///< OADDR2: a second 7-bit address
    bool general_call = false;
};

constexpr bool i2c_address_config_valid(const I2cAddressConfig& a) {
    if (a.ten_bit) {
        return a.own <= 0x3FFu && !a.second;   // ENDUAL is a 7-bit arrangement
    }
    return a.own <= 0x7Fu && (!a.second || *a.second <= 0x7Fu);
}

// =============================================================================
// The resource
// =============================================================================

/**
 * I2c<n>: the register block and its verbs.
 *
 *   using Bus = brio::I2c<1>;
 *   Bus::bus_clock(true);
 *   Bus::reset();
 *   Bus::timing(*brio::i2c_timing_for(48'000'000, brio::I2cSpeed::fast_400k));
 *   Bus::enable();
 *
 * Every verb is one of the chapter's sequences or one bit; the phases
 * are the engine's.
 */
template <uint8_t n>
struct I2c {
    static_assert(i2c_base_for(n) != 0, "brio I2c: this family has I2C1 and I2C2");
    static_assert(i2c_present(n),
                  "brio I2c: this part does not offer that instance - the CH32V203F6 has "
                  "no I2C at all and every part below the CH32V203C8 has I2C1 alone "
                  "(datasheet table 2-1, parts/<part>.hpp)");

    I2c() = delete;

    static constexpr uint8_t number = n;
    static constexpr Bus bus = i2c_bus_for(n);
    /// Table 11-5: the two channels this instance's requests reach.
    static constexpr uint8_t dma_tx_channel = i2c_dma_tx_channel(n);
    static constexpr uint8_t dma_rx_channel = i2c_dma_rx_channel(n);

    static I2cRegs& regs() { return *reinterpret_cast<I2cRegs*>(i2c_base_for(n)); }
    static constexpr Irq event_irq() { return i2c_event_irq_for(n); }
    static constexpr Irq error_irq() { return i2c_error_irq_for(n); }
    static volatile void* data_address() { return &regs().DATAR; }

    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(bus, i2c_gate_for(n));
        } else {
            Rcc::disable(bus, i2c_gate_for(n));
        }
    }
    /// The RCC reset pulse: every register back to its reset value.
    static void reset() { Rcc::reset(bus, i2c_gate_for(n)); }

    /// The instance's own bus rate out of the application's clock tag.
    template <typename C>
    static constexpr uint32_t bus_hz(C clock) {
        return i2c_bus_hz<C>(clock);
    }

    /// Put the column this instance's pads come from into AFIO. I2C2 has
    /// no remap field, so only code 0 is accepted for it.
    static bool remap(uint8_t code) {
        if constexpr (n == 1) {
            return Afio::remap(Remap::i2c1, code);
        } else {
            return code == 0u;
        }
    }

    /// The timing, with PE clear on the way in: FREQ, CKCFGR and RTR are
    /// written with the peripheral disabled (19.3's order, and 19.12.9's
    /// own rule for RTR).
    static void timing(I2cTiming t) {
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~i2c_pe);
        regs().CTLR2 =
            static_cast<uint16_t>((regs().CTLR2 & ~i2c_freq_mask) | (t.freq_mhz & i2c_freq_mask));
        regs().CKCFGR = t.ckcfgr;
        regs().RTR = static_cast<uint16_t>(t.trise & i2c_trise_mask);
    }

    /// The own addresses, refused by i2c_address_config_valid().
    static bool addresses(const I2cAddressConfig& a) {
        if (!i2c_address_config_valid(a)) {
            return false;
        }
        uint16_t o1 = 0;
        if (a.ten_bit) {
            o1 |= i2c_addmode | static_cast<uint16_t>(a.own & 0x3FFu);
        } else {
            o1 |= static_cast<uint16_t>(static_cast<uint16_t>(a.own & 0x7Fu) << 1);
        }
        regs().OADDR1 = o1;
        regs().OADDR2 =
            a.second ? static_cast<uint16_t>(i2c_endual |
                                             (static_cast<uint16_t>(*a.second & 0x7Fu) << 1))
                     : uint16_t{0};
        general_call(a.general_call);
        return true;
    }

    static void enable() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | i2c_pe); }
    static void disable() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~i2c_pe); }
    static bool enabled() { return (regs().CTLR1 & i2c_pe) != 0u; }

    /// SWRST: the state machines and the flags back to reset, the
    /// configuration kept - 19.12.1's verb for a bus that shows BUSY
    /// with no STOP in sight. Pulsed here: set, then cleared.
    static void software_reset() {
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | i2c_swrst);
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~i2c_swrst);
    }

    // ---- the control bits ----------------------------------------------------

    static void start() { ctlr1(i2c_start, true); }
    static bool starting() { return (regs().CTLR1 & i2c_start) != 0u; }
    static void stop() { ctlr1(i2c_stop, true); }
    static bool stopping() { return (regs().CTLR1 & i2c_stop) != 0u; }
    static void ack(bool on) { ctlr1(i2c_ack, on); }
    /// POS: the ACK bit governs the NEXT byte to arrive rather than the
    /// current one - the two-byte receive's arrangement.
    static void pos(bool on) { ctlr1(i2c_pos, on); }
    static void general_call(bool on) { ctlr1(i2c_engc, on); }
    static void no_stretch(bool on) { ctlr1(i2c_nostretch, on); }
    /// ENPEC: the CRC-8 over every byte of the tenure, address included.
    static void pec(bool on) { ctlr1(i2c_enpec, on); }
    /// PEC: the byte under way IS the checksum (transfer) or is to be
    /// compared against it (receive). Hardware clears it at the
    /// transfer's end.
    static void pec_next(bool on) { ctlr1(i2c_pec, on); }
    /// SMBus mode, and which kind of device this end claims to be.
    static void smbus(bool on, bool host_type = false) {
        uint16_t v = static_cast<uint16_t>(regs().CTLR1 & ~(i2c_smbus | i2c_smbtype));
        if (on) {
            v |= i2c_smbus;
        }
        if (host_type) {
            v |= i2c_smbtype;
        }
        regs().CTLR1 = v;
    }
    static void arp(bool on) { ctlr1(i2c_enarp, on); }
    /// The SMBus alert line, driven low by this end (the PAD is the
    /// application's to claim: it is not in the remap column).
    static void alert(bool on) { ctlr1(i2c_alert, on); }
    static void dma(bool on, bool last = false) {
        uint16_t v = static_cast<uint16_t>(regs().CTLR2 & ~(i2c_dmaen | i2c_last));
        if (on) {
            v |= i2c_dmaen;
        }
        if (last) {
            v |= i2c_last;
        }
        regs().CTLR2 = v;
    }

    // ---- data and status -----------------------------------------------------

    static void data(uint8_t v) { regs().DATAR = v; }
    static uint8_t data() { return static_cast<uint8_t>(regs().DATAR); }
    static uint16_t status1() { return regs().STAR1; }
    static uint16_t status2() { return regs().STAR2; }
    static bool flag(uint16_t mask) { return (regs().STAR1 & mask) != 0u; }
    static bool busy() { return (regs().STAR2 & i2c_busy) != 0u; }
    static bool host() { return (regs().STAR2 & i2c_msl) != 0u; }
    /// TRA: this side is transmitting in the tenure under way.
    static bool transmitting() { return (regs().STAR2 & i2c_tra) != 0u; }
    static bool second_address_matched() { return (regs().STAR2 & i2c_dualf) != 0u; }
    static bool general_call_matched() { return (regs().STAR2 & i2c_gencall) != 0u; }
    static bool smbus_host_matched() { return (regs().STAR2 & i2c_smbhost) != 0u; }
    static bool smbus_default_matched() { return (regs().STAR2 & i2c_smbdefault) != 0u; }
    /// STAR2's upper half: the running PEC, valid while ENPEC is set.
    static uint8_t pec_value() { return static_cast<uint8_t>(regs().STAR2 >> i2c_pec_shift); }

    /// EVT6/EVT1: ADDR is cleared by reading STAR1 then STAR2, in that
    /// order. Returns STAR2, which tells the direction (TRA) and which
    /// address matched.
    static uint16_t clear_addr() {
        (void)regs().STAR1;
        return regs().STAR2;
    }
    /// EVT4: STOPF is cleared by reading STAR1 then writing CTLR1.
    static void clear_stopf() {
        (void)regs().STAR1;
        regs().CTLR1 = regs().CTLR1;
    }
    /// The error flags are write-zero-to-clear.
    static void clear_errors(uint16_t mask) {
        regs().STAR1 = static_cast<uint16_t>(~(mask & i2c_all_errors));
    }

    // ---- interrupts ----------------------------------------------------------

    static void event_interrupt(bool on) { ctlr2(i2c_itevten, on); }
    /// ITBUFEN: TxE and RxNE reach the event vector too.
    static void buffer_interrupt(bool on) { ctlr2(i2c_itbufen, on); }
    static void error_interrupt(bool on) { ctlr2(i2c_iterren, on); }

private:
    static void ctlr1(uint16_t bit, bool on) {
        if (on) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | bit);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~bit);
        }
    }
    static void ctlr2(uint16_t bit, bool on) {
        if (on) {
            regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 | bit);
        } else {
            regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 & ~bit);
        }
    }
};

// =============================================================================
// The host engine
// =============================================================================

/// A DMA block the engines could not finish: the engine's own code, in
/// the range util/bus_master.hpp leaves to engines and above the four
/// i2c_bus.hpp holds.
inline constexpr uint8_t i2c_dma_fault = bus_engine_status + 4;

/**
 * I2cHost<n, pins, TxEngine, RxEngine>
 *
 * The engine I2cBus (util/i2c_bus.hpp = BusMaster) drives. One Request
 * is one bus tenure: START, the address, tx_len bytes written, then - if
 * rx_len is not zero - a repeated START, the address again with the read
 * bit and rx_len bytes taken with the last one NACKed, then STOP. Both
 * spans empty is the PROBE: START, address, STOP - the ACK on the
 * address is the answer. Every outcome the wire can give comes back as
 * i2c_bus.hpp's codes through TransferDone{status()}.
 *
 * `speed` names a row of the timing table init() solves for the bus
 * clock; a speed the clock cannot produce is answered i2c_rejected
 * inside start(), no byte moved - the one synchronous completion of an
 * I2C engine, and the arbiter replies with status() for it.
 *
 * THE ENGINE SLOTS: the instance's own two channels, both or neither. A
 * write phase runs on the transmit engine; a read phase of two bytes or
 * more on the receive engine under CTLR2.LAST; the one-byte read on the
 * pump.
 */
template <uint8_t n, I2cPins pins = i2c_default_pins<n>, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class I2cHost {
    using S = I2c<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from ch32v203/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio I2cHost: name both DMA engines or neither - a write-then-read "
                  "tenure needs both directions inside one tenure");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "brio I2cHost: the two engines must ride two different DMA channels");
    static_assert([] {
        if constexpr (TxEngine::present) {
            return TxEngine::channel == S::dma_tx_channel && RxEngine::channel == S::dma_rx_channel;
        } else {
            return true;
        }
    }(), "brio I2cHost: on this family the channel IS the request (RM table 11-5) - I2C1 "
         "transmits on DMA channel 6 and receives on 7, I2C2 on 4 and 5");
    static_assert(i2c_pins_valid(n, pins),
                  "brio I2cHost: these I2C pads are not a bus on this part - SCL and SDA must "
                  "both be named, distinct and BONDED, and the remap code must be a column "
                  "the instance has (afio.hpp's table 10-34)");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;

public:
    I2cHost() = delete;

    using Resource = S;
    static constexpr I2cPins pin_pads = pins;
    static constexpr bool has_engines = TxEngine::present;

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

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the instance up as a bus host. `clock` is the app's Clock
    /// tag, whose PB1 rate is the one truth of the bus clock. `duty` is
    /// fast mode's shape and `rise_ns` the WIRE's rise time (0 = each
    /// mode's own maximum). Both speeds are solved here and at rebase();
    /// one the clock cannot produce is marked unreachable (speed_ok())
    /// and a Request naming it is answered i2c_rejected. False when not
    /// even standard_100k is legal - which is every PB1 outside
    /// 19.12.2's 4..60 MHz window - or when the part's AFIO refuses the
    /// column.
    template <typename Clock>
    static bool init(Clock clock, I2cDuty duty = I2cDuty::ratio_2, uint16_t rise_ns = 0) {
        static_assert(clock_follows<Clock, I2cHost>(),
                      "this I2cHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its timing table would go stale on a "
                      "clock change");
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        duty_ = duty;
        rise_ns_ = rise_ns;
        S::bus_clock(true);
        S::reset();
        if (!S::remap(pins.remap)) {
            return false;
        }
        rebase(clock_hz(clock));
        if (!valid_[0]) {
            return false;
        }
        applied_ = I2cSpeed::standard_100k;
        S::timing(table_[0]);
        S::enable();
        S::ack(false);
        if constexpr (has_engines) {
            TxEngine::arm(S::data_address());
            RxEngine::arm(S::data_address());
        }
        // The pads go to the peripheral only now. OPEN DRAIN is the
        // definition of the bus, and this family's pad has no pull in
        // any output mode (pin.hpp): the far end's pull-ups own the idle
        // level.
        SclPin::function(PinDrive::open_drain);
        SdaPin::function(PinDrive::open_drain);
        status_ = i2c_ok;
        phase_ = Phase::idle;
        S::event_interrupt(true);
        S::error_interrupt(true);
        S::buffer_interrupt(false);
        Pfic::enable(S::event_irq());
        Pfic::enable(S::error_irq());
        return true;
    }

    /// The HCLK changed (DynamicClock fan-out): the bus rate is derived
    /// from it, both rows solved again and the applied one rewritten.
    /// THE BUS MUST BE IDLE.
    static void rebase(uint32_t hz) {
        pclk_hz_ = i2c_bus_hz_at(hz);
        for (uint8_t i = 0; i < i2c_speed_count; ++i) {
            const auto t =
                i2c_timing_for(pclk_hz_, static_cast<I2cSpeed>(i), duty_, rise_ns_);
            valid_[i] = t.has_value();
            table_[i] = t.value_or(I2cTiming{});
        }
        delay_rate_ = delay_rate(hz);
        if (S::enabled() && valid_[static_cast<uint8_t>(applied_)]) {
            S::timing(table_[static_cast<uint8_t>(applied_)]);
            S::enable();
        }
    }

    static bool speed_ok(I2cSpeed s) { return valid_[static_cast<uint8_t>(s)]; }
    static I2cTiming timing_of(I2cSpeed s) { return table_[static_cast<uint8_t>(s)]; }
    static uint32_t scl_hz(I2cSpeed s) {
        return i2c_scl_hz(pclk_hz_, table_[static_cast<uint8_t>(s)]);
    }
    static uint32_t reference_hz() { return pclk_hz_; }

    // ---- the transfer -------------------------------------------------------

    /// Begin a tenure (I2cBus calls it from main context). False
    /// whenever the wire moves: the tenure completes on the event vector
    /// and a TransferDone{status()} follows. True only for the one
    /// refusal that moves nothing - a speed the clock cannot produce,
    /// answered i2c_rejected through status() - the I2cHost contract
    /// (docs/design/i2c-bus.md).
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        if (!speed_ok(r.speed)) {
            status_ = i2c_rejected;
            phase_ = Phase::idle;
            return true;
        }
        status_ = i2c_ok;
        phase_ = (r.tx_len == 0u && r.rx_len != 0u) ? Phase::start_rx : Phase::start_tx;
        // The last tenure's STOP may still be on its way out: STOP stands
        // in CTLR1 until the condition is on the wire, and a client that
        // stretches the clock after the last acknowledge holds it there.
        // CTLR1 must not be written while STOP stands - a second STOP
        // request otherwise, the F1 lineage's rule - so it is waited for
        // before anything writes CTLR1, a speed change's PE cycle
        // included, BOUNDED IN TIME (stop_drain_us). A clock held past
        // the bound is a wedge: the tenure PARKS - no START and no vector
        // - for the per-bus timeout to answer.
        if (!wait_for_us([] { return !S::stopping(); }, stop_drain_us)) {
            return false;
        }
        // BUSY clears a moment after our STOP is seen; a bus still busy
        // after bus_free_us is someone else's. A START into a busy bus is
        // held by the hardware until the bus frees, which is where two
        // controllers meet and the arbitration decides - so the wait is
        // not an error, and what follows it is.
        if (!wait_for_us([] { return !S::busy(); }, bus_free_us)) {
            (void)unwedge_busy();
        }
        apply(r.speed);
        S::pos(false);
        S::ack(false);
        S::buffer_interrupt(false);
        S::start();
        return false;
    }

    /**
     * 19.12.1'S OWN CASE, AND THE ONE VERB THAT ANSWERS IT: SWRST "can
     * reset the I2C module when no stop condition is detected on the bus
     * but the busy bit is 1". That state is real on this silicon - a
     * DMA-served read has been seen to end with the wire idle, both lines
     * high, and BUSY standing (docs/ch32v203/i2c.md) - and a START issued
     * into it is held by the hardware for ever, because the hardware is
     * waiting for a bus that is already free.
     *
     * So: when the bounded wait for BUSY runs out AND THE WIRE SAYS IDLE,
     * the bit is the peripheral's own and the chapter's reset is taken.
     * The wire is what guards it: a bus another master is really holding
     * must never be reset out from under it, and there both lines are not
     * high. True when the machine was reset.
     */
    static bool unwedge_busy() {
        if (!S::busy() || !SclPin::read() || !SdaPin::read()) {
            return false;
        }
        S::software_reset();
        // SWRST TAKES CTLR2 WITH IT, the two interrupt enables included:
        // the whole configuration is written again, or the tenure that
        // follows would run with no vector at all.
        S::timing(table_[static_cast<uint8_t>(applied_)]);
        S::enable();
        S::ack(false);
        S::buffer_interrupt(false);
        S::event_interrupt(true);
        S::error_interrupt(true);
        ++unwedged_;
        return true;
    }

    /// How many times the bit above has been reset out of the way - a
    /// count, not a promise, and what a suite prints to say the case is
    /// real on this silicon.
    static uint16_t unwedges() { return unwedged_; }

    /// The engine's completion status, for the TransferDone payload.
    static uint8_t status() { return status_; }

    /// Is a tenure in flight? What a sleep site asks on a family whose
    /// bus matrix serves the core alone while it sleeps.
    static bool busy() { return phase_ != Phase::idle; }

    /// The EVENT vector's body - call from the instance's event handler.
    /// Returns true when the tenure just completed: the edge the app's
    /// glue posts TransferDone on.
    [[gnu::always_inline]] static bool isr() {
        const uint16_t s1 = S::status1();
        // STOPF is the CLIENT half's flag - a STOP seen after a START
        // this host did not issue: another master's, or the one unstick()
        // makes by hand - and ITEVTEN routes it here in every phase. Left
        // standing it re-enters this vector without end (measured on the
        // CH32V00x: the storm after an unstick). Its sequence ends in a
        // CTLR1 write, which must not happen while START or STOP stands,
        // so it waits for them to leave - a bit time at most.
        if ((s1 & i2c_stopf) != 0u && !S::starting() && !S::stopping()) {
            S::clear_stopf();
        }
        switch (phase_) {
            case Phase::idle:
                // Nothing in flight: an ADDR here is the client half's
                // too, and its sequence (STAR1 read above, then STAR2)
                // writes nothing.
                if ((s1 & i2c_addr) != 0u) {
                    (void)S::status2();
                }
                return false;

            case Phase::start_tx:
            case Phase::start_rx:
                if ((s1 & i2c_sb) == 0u) {
                    return false;
                }
                // EVT5: the address, with the direction bit.
                if (phase_ == Phase::start_rx) {
                    prime_receive();
                    S::data(static_cast<uint8_t>((req_.addr << 1) | 1u));
                    phase_ = Phase::addr_rx;
                } else {
                    S::data(static_cast<uint8_t>(req_.addr << 1));
                    phase_ = Phase::addr_tx;
                }
                return false;

            case Phase::addr_tx:
                if ((s1 & i2c_addr) == 0u) {
                    return false;
                }
                if (req_.tx_len == 0u) {
                    // The probe: acknowledged, and that was the question.
                    (void)S::clear_addr();
                    S::stop();
                    return finish(i2c_ok);
                }
                if constexpr (has_engines) {
                    S::dma(true, false);
                    (void)TxEngine::start(req_.tx.get(), req_.tx_len);
                    (void)S::clear_addr();
                    phase_ = Phase::tx_dma;
                    return false;
                }
                (void)S::clear_addr();
                phase_ = Phase::tx;
                S::buffer_interrupt(true);   // EVT8_1 follows at once
                return false;

            case Phase::tx:
                if ((s1 & i2c_txe) != 0u && pos_ < req_.tx_len) {
                    S::data(req_.tx.get()[pos_]);
                    ++pos_;
                    if (pos_ >= req_.tx_len) {
                        S::buffer_interrupt(false);   // BTF carries the end
                    }
                    return false;
                }
                if ((s1 & i2c_btf) != 0u && pos_ >= req_.tx_len) {
                    return end_of_write();   // EVT8_2
                }
                return false;

            case Phase::tx_dma:
                // The engine loaded every byte; BTF says the last one is
                // out on the wire.
                if ((s1 & i2c_btf) != 0u && !dma_busy()) {
                    S::dma(false, false);
                    return end_of_write();
                }
                return false;

            case Phase::addr_rx:
                if ((s1 & i2c_addr) == 0u) {
                    return false;
                }
                return begin_receive();

            case Phase::rx:
                return receive_step(s1);

            case Phase::rx_dma:
                return false;   // dma_isr() ends it
        }
        return false;
    }

    /// The ERROR vector's body - call from the instance's error handler.
    /// Every error ends the tenure with its i2c_bus.hpp code; a NACK is
    /// on the address or on a byte by the phase it landed in. True when
    /// the tenure just completed.
    [[gnu::always_inline]] static bool error_isr() {
        const uint16_t s1 = S::status1();
        const uint16_t errs = s1 & i2c_all_errors;
        if (errs == 0u) {
            return false;
        }
        S::clear_errors(errs);
        if (phase_ == Phase::idle) {
            return false;
        }
        uint8_t st = i2c_bus_error;
        if ((errs & i2c_af) != 0u) {
            const bool on_address = phase_ == Phase::addr_tx || phase_ == Phase::addr_rx ||
                                    phase_ == Phase::start_tx || phase_ == Phase::start_rx;
            st = on_address ? i2c_nack_addr : i2c_nack_data;
            S::stop();   // 19.5.2: the host must generate the STOP
        } else if ((errs & i2c_arlo) != 0u) {
            st = i2c_arb_lost;   // the hardware released the bus already
        } else {
            S::stop();   // BERR in master mode: 19.5.1 leaves the abort to us
        }
        if constexpr (has_engines) {
            put_engines_away();
        }
        return finish(st);
    }

    /// The DMA channels' interrupt body - call from BOTH channels'
    /// vectors. The receive block completing ends a read phase (STOP
    /// after it); the transmit block completing is only half the story
    /// (BTF on the event vector says the last byte is out). A transfer
    /// error ends the tenure with i2c_dma_fault. True when the tenure
    /// just completed.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint8_t tx = TxEngine::service();
            if ((tx & TxEngine::flag_error) != 0u) {
                put_engines_away();
                S::stop();
                return finish(i2c_dma_fault);
            }
            if ((tx & TxEngine::flag_complete) != 0u) {
                (void)TxEngine::complete();
            }
            const uint8_t rx = RxEngine::service();
            if ((rx & RxEngine::flag_error) != 0u) {
                put_engines_away();
                S::stop();
                return finish(i2c_dma_fault);
            }
            if ((rx & RxEngine::flag_complete) != 0u && phase_ == Phase::rx_dma) {
                S::dma(false, false);
                S::stop();
                return finish(i2c_ok);
            }
        }
        return false;
    }

    /// The classic bus unstick: nine SCL pulses and a STOP, by hand,
    /// open-drain, with the pads reclaimed from the peripheral for the
    /// duration. RECOVER() FIXES THE PERIPHERAL, THIS FIXES THE WIRE.
    /// Returns the pulses it took a stuck client to release SDA (0 = the
    /// wire was never stuck), or 0xFF when nine pulses and a STOP left
    /// SDA low - a short, not a client. A healthy wire is left alone.
    static uint8_t unstick() {
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        SclPin::output(true, PinDrive::open_drain);
        SdaPin::output(true, PinDrive::open_drain);

        uint8_t released_at = 0xFF;
        bool free_now = SdaPin::read();
        if (!free_now) {
            uint8_t pulses = 0;
            for (uint8_t i = 0; i < 9u && released_at == 0xFF; ++i) {
                SclPin::clear();
                spin_half_bit();
                SclPin::set();
                spin_half_bit();
                ++pulses;
                if (SdaPin::read()) {
                    released_at = pulses;
                }
            }
            // A STOP: SDA low while SCL is high, then SDA released.
            SdaPin::clear();
            spin_half_bit();
            SclPin::set();
            spin_half_bit();
            SdaPin::set();
            spin_half_bit();
            free_now = SdaPin::read();
        } else {
            released_at = 0;
        }

        SclPin::function(PinDrive::open_drain);
        SdaPin::function(PinDrive::open_drain);
        S::clear_errors(i2c_all_errors);
        Pfic::enable(S::event_irq());
        Pfic::enable(S::error_irq());
        if (!free_now) {
            return 0xFF;
        }
        return released_at == 0xFF ? 0u : released_at;
    }

    /// Put the ENGINE back where start() is legal - the verb a timed
    /// I2cBus calls on a tenure that never answered: the engines put
    /// away, SWRST pulsed (the chapter's own verb for a controller stuck
    /// BUSY), the timing rewritten, PE back. The WIRE stays the
    /// application's (unstick()).
    static bool recover() {
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        if constexpr (has_engines) {
            put_engines_away();
        }
        phase_ = Phase::idle;
        S::buffer_interrupt(false);
        S::software_reset();
        S::timing(table_[static_cast<uint8_t>(applied_)]);
        S::enable();
        S::ack(false);
        S::event_interrupt(true);
        S::error_interrupt(true);
        Pfic::enable(S::event_irq());
        Pfic::enable(S::error_irq());
        return true;
    }

    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        S::disable();
        S::bus_clock(false);
        SclPin::release();
        SdaPin::release();
    }

private:
    enum class Phase : uint8_t {
        idle, start_tx, start_rx, addr_tx, addr_rx, tx, tx_dma, rx, rx_dma,
    };

    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio I2cHost: the DMA engines must carry uint8_t elements");

    static bool finish(uint8_t st) {
        status_ = st;
        phase_ = Phase::idle;
        S::buffer_interrupt(false);
        return true;
    }

    /// EVT8_2: the last written byte is out. A repeated START opens the
    /// read half, or the STOP ends the tenure.
    static bool end_of_write() {
        if (req_.rx_len != 0u) {
            phase_ = Phase::start_rx;
            S::start();
            return false;
        }
        S::stop();
        return finish(i2c_ok);
    }

    /// The ACK/POS arrangement the read phase's length wants, set BEFORE
    /// the address goes out (19.3's receive procedures).
    static void prime_receive() {
        if (req_.rx_len == 2u && !dma_serves_rx()) {
            S::pos(true);
            S::ack(true);
        } else {
            S::ack(req_.rx_len != 1u);
        }
    }

    /// EVT6 on a read: the procedure by count.
    static bool begin_receive() {
        if constexpr (has_engines) {
            if (dma_serves_rx()) {
                S::dma(true, true);   // LAST: the block's last byte is NACKed
                (void)RxEngine::start(req_.rx.get(), req_.rx_len);
                (void)S::clear_addr();
                phase_ = Phase::rx_dma;
                return false;
            }
        }
        pos_ = 0;
        if (req_.rx_len == 1u) {
            // ACK is already clear (prime_receive): clear ADDR, then STOP
            // at once; the byte lands on RxNE.
            (void)S::clear_addr();
            S::stop();
            phase_ = Phase::rx;
            S::buffer_interrupt(true);
            return false;
        }
        if (req_.rx_len == 2u) {
            // POS and ACK were set: clear ADDR, then ACK off - the NACK
            // lands on the second byte; one BTF delivers both.
            (void)S::clear_addr();
            S::ack(false);
            phase_ = Phase::rx;
            return false;
        }
        (void)S::clear_addr();
        phase_ = Phase::rx;
        S::buffer_interrupt(req_.rx_len > 3u);   // RxNE while more than three remain
        return false;
    }

    /// The receive pump, per the count's procedure.
    static bool receive_step(uint16_t s1) {
        uint8_t* out = req_.rx.get();
        if (req_.rx_len == 1u) {
            if ((s1 & i2c_rxne) == 0u) {
                return false;
            }
            out[0] = S::data();
            return finish(i2c_ok);
        }
        if (req_.rx_len == 2u) {
            if ((s1 & i2c_btf) == 0u) {
                return false;
            }
            S::stop();
            out[0] = S::data();
            out[1] = S::data();
            S::pos(false);
            return finish(i2c_ok);
        }
        const uint8_t remaining = static_cast<uint8_t>(req_.rx_len - pos_);
        if (remaining > 3u) {
            if ((s1 & i2c_rxne) != 0u) {
                out[pos_] = S::data();
                ++pos_;
                if (req_.rx_len - pos_ == 3u) {
                    S::buffer_interrupt(false);   // BTF from here on
                }
            }
            return false;
        }
        if ((s1 & i2c_btf) == 0u) {
            return false;
        }
        if (remaining == 3u) {
            // N-2 in DATAR, N-1 in the shifter: NACK the one to come,
            // read N-2, which lets N in.
            S::ack(false);
            out[pos_] = S::data();
            ++pos_;
            return false;
        }
        // remaining == 2: N-1 in DATAR, N in the shifter.
        S::stop();
        out[pos_] = S::data();
        ++pos_;
        (void)wait_until([] { return S::flag(i2c_rxne); });
        out[pos_] = S::data();
        ++pos_;
        return finish(i2c_ok);
    }

    static bool dma_serves_rx() {
        if constexpr (has_engines) {
            return req_.rx_len >= 2u;
        } else {
            return false;
        }
    }
    static bool dma_busy() {
        if constexpr (has_engines) {
            return TxEngine::busy();
        } else {
            return false;
        }
    }
    static void put_engines_away() {
        if constexpr (has_engines) {
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address());
            S::dma(false, false);
        }
    }

    static void apply(I2cSpeed s) {
        if (s == applied_) {
            return;
        }
        applied_ = s;
        S::timing(table_[static_cast<uint8_t>(s)]);
        S::enable();
    }

    /// How long start() waits for the last tenure's STOP to leave the
    /// wire. A STOP takes a bit period; what can hold it longer is a slow
    /// client stretching the clock after the last acknowledge while it
    /// digests a write, and five milliseconds - a fifth of SMBus's 25 ms
    /// clock-low limit - is the bound. What start() spends here is the
    /// kernel dispatch's time, which is why it is bounded at all.
    static constexpr uint16_t stop_drain_us = 5'000;

    /// How long start() then waits for BUSY to clear: our STOP seen by
    /// the bus takes a moment, another device's traffic takes longer.
    static constexpr uint16_t bus_free_us = 100;

    /// Poll `pred` for at least `us` microseconds, timed on the STK
    /// (delay.hpp); true as soon as it holds. delay_us() refuses when the
    /// STK is not running, and the bound is then a count of polls - a
    /// bound still.
    template <typename Pred>
    static bool wait_for_us(Pred pred, uint16_t us) {
        for (uint16_t t = 0; t < us; ++t) {
            if (pred()) {
                return true;
            }
            (void)delay_us(delay_rate_, 1);
        }
        return pred();
    }

    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }

    /// Half a standard-mode bit, for the unstick.
    static void spin_half_bit() { (void)delay_us(delay_rate_, 5); }

    static inline Request req_{};
    static inline uint8_t pos_ = 0;
    static inline volatile Phase phase_ = Phase::idle;
    static inline uint8_t status_ = i2c_ok;
    static inline I2cSpeed applied_ = I2cSpeed::standard_100k;
    static inline I2cDuty duty_ = I2cDuty::ratio_2;
    static inline uint16_t rise_ns_ = 0;
    static inline uint16_t unwedged_ = 0;
    static inline uint32_t pclk_hz_ = 0;
    static inline I2cTiming table_[i2c_speed_count]{};
    static inline bool valid_[i2c_speed_count]{};
    static inline DelayRate delay_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/// What the client's ISR body reports: one thing per call, the app's
/// glue acts on it.
enum class I2cClientEvent : uint8_t {
    none,
    addressed,       ///< ADDR: a tenure opened on one of the own addresses
    byte_received,   ///< RxNE: take() it
    byte_wanted,     ///< TxE: give() the next
    stop,            ///< STOPF: the tenure ended
    nacked,          ///< AF: the host refused a byte this client gave - the end of a read
    error,           ///< BERR, ARLO or OVR
};

/// What a client is beyond its addresses. `interrupts` false leaves the
/// three enables down for a POLLED client, which is what an instrument
/// that must spend a commanded hold inside a tenure needs: the flags
/// then belong to the loop alone.
struct I2cClientOptions {
    bool no_stretch = false;
    bool interrupts = true;
};

/**
 * I2cClient<n, pins>
 *
 * The target side. The peripheral matches the own address(es), ACKs it,
 * raises ADDR and STRETCHES SCL until the sequence that clears it runs
 * (19.4); every byte then comes on RxNE or is asked for on TxE, with the
 * clock stretched while the flag stands unless NOSTRETCH. A read tenure
 * ends with the host's NACK (AF, on the error vector); a write tenure
 * with STOPF.
 *
 *   extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() { act(Peer::service()); }
 *   extern "C" BRIO_CH32_INTERRUPT void i2c1_er_handler() { act(Peer::error_service()); }
 */
template <uint8_t n, I2cPins pins = i2c_default_pins<n>>
class I2cClient {
    using S = I2c<n>;

    static_assert(i2c_pins_valid(n, pins),
                  "brio I2cClient: these I2C pads are not a bus on this part - SCL and SDA "
                  "must both be named, distinct and BONDED, and the remap code must be a "
                  "column the instance has (afio.hpp's table 10-34)");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;

public:
    I2cClient() = delete;

    using Resource = S;
    static constexpr I2cPins pin_pads = pins;

    /// Bring the instance up as a bus target. FREQ must still state the
    /// bus clock (the block generates its data setup and hold times from
    /// it even as a target), so 19.12.2's window applies here too and a
    /// PB1 outside it is refused.
    template <typename Clock>
    static bool init(Clock clock, const I2cAddressConfig& addr,
                     const I2cClientOptions& opts = {}) {
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        S::bus_clock(true);
        S::reset();
        if (!S::remap(pins.remap)) {
            return false;
        }
        const auto t = i2c_timing_for(i2c_bus_hz(clock), I2cSpeed::standard_100k);
        if (!t) {
            return false;
        }
        S::timing(*t);
        if (!S::addresses(addr)) {
            return false;
        }
        S::no_stretch(opts.no_stretch);
        S::enable();
        S::ack(true);   // answer the own address
        SclPin::function(PinDrive::open_drain);
        SdaPin::function(PinDrive::open_drain);
        S::event_interrupt(opts.interrupts);
        S::buffer_interrupt(opts.interrupts);
        S::error_interrupt(opts.interrupts);
        if (opts.interrupts) {
            Pfic::enable(S::event_irq());
            Pfic::enable(S::error_irq());
        }
        return true;
    }

    // ---- the protocol surface, polled -----------------------------------------

    /// An address match is pending (SCL held meanwhile, unless
    /// NOSTRETCH).
    static bool addressed() { return S::flag(i2c_addr); }
    /// Answer the match and release the clock: EVT1's STAR1-then-STAR2
    /// read. Returns true when the host READS (this side transmits).
    static bool answer_address() { return (S::clear_addr() & i2c_tra) != 0u; }
    /// Valid after answer_address(): which own address matched.
    static bool second_address_matched() { return S::second_address_matched(); }
    static bool general_call_matched() { return S::general_call_matched(); }
    /// Host write: a byte has arrived.
    static bool data_ready() { return S::flag(i2c_rxne); }
    static uint8_t take() { return S::data(); }
    /// Host read: the shifter wants the next byte.
    static bool data_wanted() { return S::flag(i2c_txe); }
    static void give(uint8_t v) { S::data(v); }
    /// Whether to ACK the bytes that arrive from here on.
    static void acknowledge(bool on) { S::ack(on); }
    /// A STOP arrived - the end of a write tenure.
    static bool stop_seen() { return S::flag(i2c_stopf); }
    static void clear_stop() { S::clear_stopf(); }
    /// The host refused a byte this client gave: the end of a read tenure
    /// (19.5.2: the hardware released the bus).
    static bool host_nacked() { return S::flag(i2c_af); }
    static void clear_nack() { S::clear_errors(i2c_af); }
    static bool overrun() { return S::flag(i2c_ovr); }
    static void clear_overrun() { S::clear_errors(i2c_ovr); }
    static uint16_t flags() { return S::status1(); }

    /**
     * THROW AWAY A BYTE THE HOST NEVER TOOK. A target transmitter is
     * asked for one byte more than the controller takes: DATAR empties
     * into the shifter and TxE rises again, so the byte standing in
     * DATAR when the closing NACK arrives is never clocked out and would
     * be the FIRST byte of the next read tenure. The chapter has no verb
     * that discards it; a PE cycle does, because clearing PE resets the
     * state machines, the communication control bits and the status bits
     * while CTLR2, OADDR1, OADDR2, CKCFGR and RTR keep what they hold -
     * so the address and the timing survive and only the loaded byte is
     * lost. ACK is a control bit and goes back up here.
     *
     * True when there was a byte to drop; false when DATAR was already
     * empty (TxE standing). THE FLAG CANNOT TELL THE TWO APART ON AN
     * IDLE CLIENT of this silicon: measured, TxE reads DOWN in a client
     * nobody is addressing, with or without a byte in DATAR, so this
     * verb belongs where the question is real - the moment after the
     * host's closing NACK - and not to a program polling an idle port.
     */
    static bool flush() {
        if (S::flag(i2c_txe)) {
            return false;
        }
        S::disable();
        S::enable();
        S::ack(true);
        return true;
    }

    // ---- the ISR bodies -----------------------------------------------------

    /// The EVENT vector's body: one event per call, the flag consumed
    /// where the chapter's sequence consumes it (ADDR and STOPF here;
    /// RxNE by take(), TxE by give(), which the glue owes).
    [[gnu::always_inline]] static I2cClientEvent service() {
        const uint16_t s1 = S::status1();
        if ((s1 & i2c_addr) != 0u) {
            host_reads_ = answer_address();
            return I2cClientEvent::addressed;
        }
        if ((s1 & i2c_stopf) != 0u) {
            S::clear_stopf();
            return I2cClientEvent::stop;
        }
        if ((s1 & i2c_rxne) != 0u) {
            return I2cClientEvent::byte_received;
        }
        if ((s1 & i2c_txe) != 0u) {
            return I2cClientEvent::byte_wanted;
        }
        return I2cClientEvent::none;
    }

    /// The ERROR vector's body: the host's NACK (the normal end of a
    /// read) or a real error, the flag cleared.
    [[gnu::always_inline]] static I2cClientEvent error_service() {
        const uint16_t s1 = S::status1();
        const uint16_t errs = s1 & i2c_all_errors;
        if (errs == 0u) {
            return I2cClientEvent::none;
        }
        S::clear_errors(errs);
        return (errs & i2c_af) != 0u ? I2cClientEvent::nacked : I2cClientEvent::error;
    }

    /// The direction of the tenure the last `addressed` opened: true =
    /// the host reads, this side transmits.
    static bool host_reads() { return host_reads_; }

    static void release() {
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        S::disable();
        S::bus_clock(false);
        SclPin::release();
        SdaPin::release();
    }

private:
    static inline bool host_reads_ = false;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// At 48 MHz of PB1: standard mode CCR = 240 (48e6 / 200e3), fast mode
// CCR = 40 at DUTY 2 (48e6 / 1.2e6) and 5 at 16/9 (48e6 / 10e6 rounded
// up); TRISE = 1000 ns x 48 + 1 = 49 in standard mode and 300 ns x 48 + 1
// = 15 in fast mode.
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k)->ckcfgr == 240u);
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k)->freq_mhz == 48u);
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k)->trise == 49u);
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k)->ckcfgr == (i2c_fs | 40u));
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k)->trise == 15u);
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k, I2cDuty::ratio_16_9)->ckcfgr ==
              (i2c_fs | i2c_duty | 5u));
static_assert(i2c_scl_hz(48'000'000UL, *i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k)) ==
              100'000UL);
static_assert(i2c_scl_hz(48'000'000UL, *i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k)) ==
              400'000UL);
// A MEASURED wire: a microsecond of rise told to fast mode is TRISE = 49,
// the same number standard mode's maximum gives.
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k, I2cDuty::ratio_2, 1000)->trise ==
              49u);
// Rounded UP: at 8 MHz fast mode wants 6.67 -> 7 (381 kHz).
static_assert(i2c_timing_for(8'000'000UL, I2cSpeed::fast_400k)->ckcfgr == (i2c_fs | 7u));
static_assert(i2c_scl_hz(8'000'000UL, *i2c_timing_for(8'000'000UL, I2cSpeed::fast_400k)) ==
              380'952UL);
// The window (19.12.2): below 4 MHz and above 60 MHz nothing is legal,
// which is what refuses the stratum's own PB1 ceiling of 72 MHz.
static_assert(!i2c_timing_for(3'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(!i2c_timing_for(72'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(i2c_timing_for(60'000'000UL, I2cSpeed::standard_100k)->trise == 61u);
static_assert(i2c_timing_for(4'000'000UL, I2cSpeed::standard_100k)->ckcfgr == 20u);

// The pads of the two columns I2C1 has and the one I2C2 has (table
// 10-34 and the datasheet's pin table).
static_assert(i2c_pins_for(1, 0).scl == Pad{'B', 6} && i2c_pins_for(1, 0).sda == Pad{'B', 7});
static_assert(i2c_pins_for(1, 1).scl == Pad{'B', 8} && i2c_pins_for(1, 1).sda == Pad{'B', 9} &&
              i2c_pins_for(1, 1).remap == 1u);
static_assert(i2c_pins_for(2, 0).scl == Pad{'B', 10} && i2c_pins_for(2, 0).sda == Pad{'B', 11} &&
              i2c_pins_for(2, 0).remap == 0u);
// I2C2 has one column: a code of 1 is not a remap but a name for
// nothing.
static_assert(i2c_column_count(1) == 2u && i2c_column_count(2) == 1u);
static_assert(!i2c_pins_valid(2, I2cPins{.scl = {'B', 10}, .sda = {'B', 11}, .remap = 1}));
static_assert(!i2c_pins_valid(1, I2cPins{.scl = {'B', 6}, .sda = {'B', 6}}));
static_assert(!i2c_pins_valid(1, I2cPins{.scl = {'B', 6}}));

static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x48}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x80}));
static_assert(!i2c_address_config_valid(
    I2cAddressConfig{.own = 0x123, .ten_bit = true, .second = 0x10}));
static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x123, .ten_bit = true}));

} // namespace brio
