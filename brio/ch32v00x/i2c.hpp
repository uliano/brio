/*
 * i2c.hpp
 *
 * The I2C of the CH32V00x (RM ch. 15): one instance, host and client,
 * 7- and 10-bit addresses, dual addressing and the general call, two
 * speeds, clock stretching, PEC, two DMA requests - the STM32F1's I2C,
 * an EVENT MACHINE (SB, ADDR, TxE, RxNE, BTF, STOPF and the five
 * errors) over two vectors, one for events and one for errors. Three
 * layers, the other three strata's arrangement:
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
 * each stretching SCL low until it completes (15.3's notes): EV5 (SB
 * up: read STAR1, write the address), EV6 (ADDR up: read STAR1 then
 * STAR2), EV8 (TxE: write DATAR), EV8_2 (TxE and BTF: the last byte is
 * out, STOP or repeated START), EV7 (RxNE: read DATAR) - and a
 * RECEIVE PROCEDURE THAT DEPENDS ON THE COUNT: one byte wants ACK
 * cleared BEFORE ADDR is cleared and STOP set right after; two bytes
 * want POS set with ACK, ACK cleared after ADDR, and the pair read
 * after one BTF; three or more run on RxNE until three remain, then
 * wait for BTF twice - the first clears ACK and reads N-2, the second
 * sets STOP and reads N-1 and N. All of it is in `isr()`, phase by
 * phase, and the reason the ITBUFEN enable is switched on and off
 * during a tenure: TxE and RxNE interrupt only while the byte pump
 * needs them, BTF (under ITEVTEN alone) carries the rest. ITEVTEN is
 * the TENURE'S in turn - raised by start(), dropped at its end - because
 * the BTF a STOP leaves standing until the condition is out (15.10.6)
 * and the slave half's STOPF would otherwise enter the vector between
 * tenures for nothing; a DMA read phase drops it too, the channel's
 * completion ending that phase (begin_receive()).
 *
 * THE REPEATED START OF A WRITE-THEN-READ IS REQUESTED WHILE THE LAST
 * WRITTEN BYTE IS STILL GOING OUT, on the TxE that says it went into the
 * shifter: 15.10.1's START bit, set in master mode, makes the controller
 * repeat the start condition, and requested while a byte shifts it goes
 * out as that byte ends (measured: every written byte taken, and no BTF
 * entering the vector behind it). It is not the chapter's own end of a write - 15.3 ends a write at
 * EV8_2 (TxE and BTF, figure 15-4), with SCL held low past the last
 * acknowledge - and it is the order WCH's interrupt example uses: a
 * POLLED target of this block's lineage loses the last written byte when
 * the START comes after BTF (measured on the CH32V203C8T6 and the
 * CH32V303VCT6, docs/ch32vx03/i2c.md), and a register index followed by
 * a read is exactly that shape. The price is in the error path: the START
 * is requested before the last byte's acknowledge is known, so the phase
 * between the request and SB is its own (`restart`), and a NACK in it is
 * the byte's - i2c_nack_data. THE REPEATED START IS NOT WITHDRAWN: the
 * controller generates it after the refused byte all the same (measured:
 * left alone, START clears and SB rises a few tens of cycles after the
 * error vector sees AF), and a CTLR1 store that clears START and sets STOP
 * races that generation - won at 100 kHz, lost at 400 kHz, where it left
 * STOP standing over a master holding the bus and the next tenure stalled
 * (measured, docs/ch32v00x/i2c.md). So the refusal lets the Sr go out and
 * closes the tenure from its SB with the one sequence EV5 allows: the
 * address with the WRITE bit and STOP requested behind it - a void write,
 * Sr A P on the wire - its ADDR cleared or its own NACK taken before the
 * tenure reports i2c_nack_data. Nothing else of the controller is touched,
 * so nothing races it.
 * The write half of a write-then-read runs on the byte pump with the
 * engines too: the order needs an event when the last byte enters the
 * shifter, which the transmit block gives only through its completion
 * interrupt - and the transmit channel is armed for its errors alone, so
 * that a plain write takes no DMA interrupt at all.
 *
 * THERE IS NO RISE-TIME REGISTER, ON EITHER PART. 15.3's text names an
 * "R16_I2C1_RTR" the register list (table 15-1) does not carry - the
 * CH32V003's manual does the same in its 13.3 against its table 13-1
 * - and the vendor's own headers end the block at CKCFGR on both: the
 * SCL timing is CKCFGR alone
 * (CCR under F/S and DUTY), and FREQ in CTLR2 must state the bus clock
 * in MHz, 8 to 48 (15.10.2) - below 8 MHz no speed is legal and init()
 * refuses. The CCR arithmetic is the F1's: standard mode divides by
 * 2 x CCR, fast mode by 3 x CCR at DUTY 2 and 25 x CCR at DUTY 16/9,
 * rounded UP here so a bus never runs faster than asked.
 *
 * THE DMA REQUESTS ARE CHANNELS 6 (transmit) and 7 (receive), table
 * 8-2 of both manuals, and an engine on any other channel is refused. With engines a
 * plain write of any length and a read phase of two bytes or more run
 * on them (CTLR2.LAST makes the controller NACK the last byte a
 * receive block takes, 15.10.2); a one-byte read stays on the pump,
 * whose ACK-before-ADDR sequence the DMA path cannot express.
 *
 * NOT COVERED YET: 10-bit addressing on the host side (the resource
 * has the mode and the client matches such an address; the host's
 * header-byte sequence, EV9, has no user); PEC (the resource has the
 * enable, no tenure shape asks for it); the general call as a host
 * verb; a wake from Standby on an address match, which this family's
 * PWR chapter does not offer.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <span>
#include <type_traits>

#include "ch32v00x/afio.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/device.hpp"
#include "ch32v00x/dma_engine.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

// =============================================================================
// The registers (table 15-1): sixteen bits at a four-byte stride
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
};

inline constexpr uint32_t i2c1_base = pb1_base + 0x5400;

constexpr uint32_t i2c_base_for(uint8_t instance) {
    return instance == 1 ? i2c1_base : 0;
}

// CTLR1 (15.10.1)
inline constexpr uint16_t i2c_pe        = 1u << 0;
inline constexpr uint16_t i2c_enpec     = 1u << 5;
inline constexpr uint16_t i2c_engc      = 1u << 6;
inline constexpr uint16_t i2c_nostretch = 1u << 7;
inline constexpr uint16_t i2c_start     = 1u << 8;
inline constexpr uint16_t i2c_stop      = 1u << 9;
inline constexpr uint16_t i2c_ack       = 1u << 10;
inline constexpr uint16_t i2c_pos       = 1u << 11;
inline constexpr uint16_t i2c_pec       = 1u << 12;
inline constexpr uint16_t i2c_swrst     = 1u << 15;
// CTLR2 (15.10.2)
inline constexpr uint16_t i2c_freq_mask = 0x3Fu;
inline constexpr uint16_t i2c_iterren   = 1u << 8;
inline constexpr uint16_t i2c_itevten   = 1u << 9;
inline constexpr uint16_t i2c_itbufen   = 1u << 10;
inline constexpr uint16_t i2c_dmaen     = 1u << 11;
inline constexpr uint16_t i2c_last      = 1u << 12;
// OADDR1 (15.10.3). The F1 lineage's "bit 14 must be kept at one" is
// NOT here: 15.10.3 lists the bit as reserved and it reads back zero
// whatever is written (measured), so nothing writes it.
inline constexpr uint16_t i2c_addmode      = 1u << 15;
// OADDR2 (15.10.4)
inline constexpr uint16_t i2c_endual    = 1u << 0;
// STAR1 (15.10.6)
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
inline constexpr uint16_t i2c_errors    = i2c_berr | i2c_arlo | i2c_af | i2c_ovr | i2c_pecerr;
// STAR2 (15.10.7)
inline constexpr uint16_t i2c_msl       = 1u << 0;
inline constexpr uint16_t i2c_busy      = 1u << 1;
inline constexpr uint16_t i2c_tra       = 1u << 2;
inline constexpr uint16_t i2c_gencall   = 1u << 4;
inline constexpr uint16_t i2c_dualf     = 1u << 7;
// CKCFGR (15.10.8)
inline constexpr uint16_t i2c_ccr_mask  = 0x0FFFu;
inline constexpr uint16_t i2c_duty      = 1u << 14;
inline constexpr uint16_t i2c_fs        = 1u << 15;

// =============================================================================
// The vocabulary
// =============================================================================

/// The two speeds 15.1 names.
enum class I2cSpeed : uint8_t { standard_100k = 0, fast_400k = 1 };

inline constexpr uint8_t i2c_speed_count = 2;

constexpr uint32_t i2c_speed_hz(I2cSpeed s) {
    return s == I2cSpeed::fast_400k ? 400'000UL : 100'000UL;
}

/// Fast mode's two duty shapes (CKCFGR.DUTY): Tlow/Thigh = 2, or 16/9.
enum class I2cDuty : uint8_t { ratio_2 = 0, ratio_16_9 = 1 };

/// What CKCFGR and CTLR2.FREQ hold for one speed at one bus clock.
struct I2cTiming {
    uint8_t freq_mhz = 0;    ///< CTLR2.FREQ, the bus clock in whole MHz
    uint16_t ckcfgr = 0;     ///< CCR with F/S and DUTY
};

/// 15.10.2's window for FREQ.
inline constexpr uint32_t i2c_min_hz = 8'000'000UL;
inline constexpr uint32_t i2c_max_hz = 48'000'000UL;

/// The chapter's arithmetic (the F1 lineage's, stated by the vendor's
/// own init and not by the register description): standard mode SCL =
/// pclk / (2 x CCR), fast mode pclk / (3 x CCR) at DUTY 2 and
/// pclk / (25 x CCR) at DUTY 16/9. CCR is rounded UP - a bus at most as
/// fast as asked - with the floors the vendor keeps (4 in standard
/// mode, 1 in fast). Nullopt outside FREQ's window, or when CCR would
/// not fit its twelve bits.
constexpr std::optional<I2cTiming> i2c_timing_for(uint32_t pclk, I2cSpeed speed,
                                                  I2cDuty duty = I2cDuty::ratio_2) {
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
    return I2cTiming{static_cast<uint8_t>(pclk / 1'000'000UL),
                     static_cast<uint16_t>(bits | static_cast<uint16_t>(ccr))};
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

/// Which pads carry the two lines, and the REMAP CODE that puts the
/// peripheral on them (afio.hpp's table 7-13-1): init() writes it.
struct I2cPins {
    Pad scl{};
    Pad sda{};
    uint8_t remap = 0;
};

/// The pins of remap column `code`.
constexpr I2cPins i2c1_pins_for(uint8_t code) {
    const I2cPadSet p = afio_i2c1_pads(code);
    return I2cPins{.scl = p.scl, .sda = p.sda, .remap = code};
}

/// I2C1's default pads on the CH32V006 (DS table 2-1-1): SCL PC2, SDA
/// PC1.
inline constexpr I2cPins i2c1_default_pins = i2c1_pins_for(0);

constexpr bool i2c_pins_valid(const I2cPins& p) {
    return p.scl.valid() && p.sda.valid() && !(p.scl == p.sda) && p.remap < 8u;
}

/// A target's addresses (15.4): one 7- or 10-bit own address, an
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
 * I2c<n>: the register block and its verbs. Every verb is one of the
 * chapter's sequences or one bit; the phases are the engine's.
 */
template <uint8_t n>
struct I2c {
    static_assert(i2c_base_for(n) != 0, "brio I2c: this family has I2C1 alone");

    I2c() = delete;

    static constexpr uint8_t number = n;
    /// Table 8-2: the two channels this instance's requests reach.
    static constexpr uint8_t dma_tx_channel = 6;
    static constexpr uint8_t dma_rx_channel = 7;

    static I2cRegs& regs() { return *reinterpret_cast<I2cRegs*>(i2c_base_for(n)); }
    static constexpr Irq event_irq() { return Irq::i2c1_ev; }
    static constexpr Irq error_irq() { return Irq::i2c1_er; }
    static volatile void* data_address() { return &regs().DATAR; }

    static void bus_clock(bool on) {
        if (on) { rcc()->PB1PCENR |= rcc_pb1_i2c1; } else { rcc()->PB1PCENR &= ~rcc_pb1_i2c1; }
    }
    /// The RCC reset pulse: every register back to its reset value.
    static void reset() {
        rcc()->PB1PRSTR |= rcc_pb1_i2c1;
        rcc()->PB1PRSTR &= ~rcc_pb1_i2c1;
    }

    /// The timing, with PE clear on the way in: FREQ and CKCFGR are
    /// written with the peripheral disabled (15.3's order).
    static void timing(I2cTiming t) {
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~i2c_pe);
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~i2c_freq_mask) | (t.freq_mhz & i2c_freq_mask));
        regs().CKCFGR = t.ckcfgr;
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
        regs().OADDR2 = a.second ? static_cast<uint16_t>(i2c_endual | (static_cast<uint16_t>(*a.second & 0x7Fu) << 1))
                                 : uint16_t{0};
        general_call(a.general_call);
        return true;
    }

    static void enable() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | i2c_pe); }
    static void disable() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~i2c_pe); }
    static bool enabled() { return (regs().CTLR1 & i2c_pe) != 0u; }

    /// SWRST: the state machines and the flags back to reset, the
    /// configuration kept - 15.10.1's verb for a bus that shows BUSY
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
    static void pec(bool on) { ctlr1(i2c_enpec, on); }
    static void dma(bool on, bool last = false) {
        uint16_t v = static_cast<uint16_t>(regs().CTLR2 & ~(i2c_dmaen | i2c_last));
        if (on) { v |= i2c_dmaen; }
        if (last) { v |= i2c_last; }
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

    /// EV6/EV1: ADDR is cleared by reading STAR1 then STAR2, in that
    /// order. Returns STAR2, which tells the direction (TRA) and which
    /// address matched.
    static uint16_t clear_addr() {
        (void)regs().STAR1;
        return regs().STAR2;
    }
    /// EV4: STOPF is cleared by reading STAR1 then writing CTLR1.
    static void clear_stopf() {
        (void)regs().STAR1;
        regs().CTLR1 = regs().CTLR1;
    }
    /// The five error flags are write-zero-to-clear.
    static void clear_errors(uint16_t mask) {
        regs().STAR1 = static_cast<uint16_t>(~(mask & i2c_errors));
    }

    // ---- interrupts ----------------------------------------------------------

    /// CTLR1's control bits, some set and some cleared, in ONE store
    /// (i2c_start, i2c_stop, i2c_ack, i2c_pos ...): what a tenure's start
    /// writes at once. No CTLR1 write may land while START or STOP stands
    /// (the F1 lineage's rule, 15.10.1), and this is one write.
    static void control(uint16_t set, uint16_t clear) {
        regs().CTLR1 = static_cast<uint16_t>((regs().CTLR1 & ~clear) | set);
    }
    /// CTLR2's interrupt enables the same way (i2c_itevten, i2c_itbufen,
    /// i2c_iterren): one store.
    static void interrupts(uint16_t set, uint16_t clear) {
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~clear) | set);
    }

    static void event_interrupt(bool on) { ctlr2(i2c_itevten, on); }
    /// ITBUFEN: TxE and RxNE reach the event vector too.
    static void buffer_interrupt(bool on) { ctlr2(i2c_itbufen, on); }
    static void error_interrupt(bool on) { ctlr2(i2c_iterren, on); }

private:
    static void ctlr1(uint16_t bit, bool on) {
        if (on) { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | bit); }
        else { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~bit); }
    }
    static void ctlr2(uint16_t bit, bool on) {
        if (on) { regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 | bit); }
        else { regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 & ~bit); }
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
 * The engine I2cBus (util/i2c_bus.hpp = BusMaster) drives. One
 * Request is one bus tenure: START, the address, tx_len bytes written,
 * then - if rx_len is not zero - a repeated START, the address again
 * with the read bit and rx_len bytes taken with the last one NACKed,
 * then STOP. Both spans empty is the PROBE: START, address, STOP - the
 * ACK on the address is the answer. Every outcome the wire can give
 * comes back as i2c_bus.hpp's codes through TransferDone{status()}.
 *
 * `speed` names a row of the timing table init() solves for the bus
 * clock; a speed the clock cannot produce is answered i2c_rejected
 * inside start(), no byte moved - the one synchronous completion of
 * an I2C engine, and the arbiter replies with status() for it.
 *
 * THE ENGINE SLOTS: `DmaTxEngine<6>` and `DmaRxEngine<7>`, both or
 * neither. A plain write runs on the transmit engine; a read phase of
 * two bytes or more on the receive engine under CTLR2.LAST; the one-
 * byte read and the write half of a write-then-read on the pump (the
 * file header: its repeated START wants the TxE of the last byte). The
 * transmit engine is armed for its ERRORS ALONE: a plain write ends on
 * BTF, which the event vector takes anyway, with the channel's count at
 * zero - the controller wrote the last byte a byte time before it left
 * the shifter - so the block's own completion interrupt would prove
 * nothing BTF does not.
 */
template <uint8_t n, I2cPins pins = i2c1_default_pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class I2cHost {
    using S = I2c<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from ch32v00x/dma.hpp, or NoDmaEngine (the default)");
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
    }(), "brio I2cHost: on this family the channel IS the request (RM table 8-2) - I2C1 "
         "transmits on DMA channel 6 and receives on channel 7");
    static_assert(i2c_pins_valid(pins),
                  "brio I2cHost: these I2C pads are not a bus - SCL and SDA must both be "
                  "named and distinct");

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
    /// tag, the one truth of the bus clock (HCLK: no APB prescaler on
    /// this family). `duty` is fast mode's shape. Both speeds are solved
    /// here and at rebase(); one the clock cannot produce is marked
    /// unreachable (speed_ok()) and a Request naming it is answered
    /// i2c_rejected. False when not even standard_100k is legal - which
    /// is every clock below 8 MHz (15.10.2).
    template <typename Clock>
    static bool init(Clock clock, I2cDuty duty = I2cDuty::ratio_2) {
        static_assert(clock_follows<Clock, I2cHost>(),
                      "this I2cHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its timing table would go stale on a "
                      "clock change");
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        duty_ = duty;
        S::bus_clock(true);
        S::reset();
        Afio::remap_i2c1(pins.remap);
        rebase(clock_hz(clock));
        if (!valid_[0]) {
            return false;
        }
        applied_ = I2cSpeed::standard_100k;
        S::timing(table_[0]);
        S::enable();
        S::ack(false);
        if constexpr (has_engines) {
            // BTF ends a write phase: the transmit block's errors alone.
            // Both at HIGH (DmaPriority's rule): the host does not overrun -
            // a byte a starved channel has not moved leaves BTF up and SCL
            // held low until it is, stretching the bus and losing nothing
            // (15.3's notes).
            TxEngine::arm(S::data_address(), TxEngine::flag_error, DmaPriority::high);
            RxEngine::arm(S::data_address(), DmaPriority::high);
        }
        // The pads go to the peripheral only now. OPEN DRAIN is the
        // definition of the bus: the pull-ups own the idle level.
        SclPin::function(PinDrive::open_drain);
        SdaPin::function(PinDrive::open_drain);
        status_ = i2c_ok;
        phase_ = Phase::idle;
        // At rest the error line alone: the event line is a tenure's,
        // raised by its start() and dropped by its end (the class
        // comment).
        S::interrupts(i2c_iterren, i2c_itevten | i2c_itbufen);
        Pfic::enable(S::event_irq());
        Pfic::enable(S::error_irq());
        return true;
    }

    /// The bus clock changed (DynamicClock fan-out): both rows solved
    /// again and the applied one rewritten. THE BUS MUST BE IDLE.
    static void rebase(uint32_t hz) {
        pclk_hz_ = hz;
        for (uint8_t i = 0; i < i2c_speed_count; ++i) {
            const auto t = i2c_timing_for(hz, static_cast<I2cSpeed>(i), duty_);
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
    static uint32_t scl_hz(I2cSpeed s) { return i2c_scl_hz(pclk_hz_, table_[static_cast<uint8_t>(s)]); }
    static uint32_t reference_hz() { return pclk_hz_; }

    // ---- the transfer -------------------------------------------------------

    /// Begin a tenure (I2cBus calls it from main context). False
    /// whenever the wire moves: the tenure completes on the event
    /// vector and a TransferDone{status()} follows. True only for the
    /// one refusal that moves nothing - a speed the clock cannot
    /// produce, answered i2c_rejected through status() - the I2cHost
    /// contract (docs/design/i2c-bus.md).
    static bool start(const Request& r) {
        // What the tenure's entries read, and nothing else of the Request:
        // the reply and the speed are the arbiter's and start()'s.
        t_.tx = r.tx;
        t_.rx = r.rx;
        t_.addr = r.addr;
        t_.tx_len = r.tx_len;
        t_.rx_len = r.rx_len;
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
        // after bus_free_us is someone else's. On this silicon a START
        // into a busy bus does not wait for it to free: measured with a
        // peer holding SDA low, the START is answered ARLO, and
        // i2c_arb_lost is the reply - the wire's own answer, in its place,
        // inside any per-bus timeout.
        (void)wait_for_us([] { return !S::busy(); }, bus_free_us);
        apply(r.speed);
        // ONE CTLR1 store: START, with POS and ACK down for the address
        // phase. Read after STAR1, it is also the second half of a stale
        // STOPF's clear (15.10.6: STAR1 read, then a CTLR1 write) - the
        // slave half's, which nothing between two tenures takes, the
        // event line being down at rest. Then the tenure's event line.
        (void)S::status1();
        S::control(i2c_start, i2c_pos | i2c_ack);
        S::interrupts(i2c_itevten, i2c_itbufen);
        return false;
    }

    /// The engine's completion status, for the TransferDone payload.
    static uint8_t status() { return status_; }

    /// The EVENT vector's body - call from i2c1_ev's handler. Returns
    /// true when the tenure just completed: the edge the app's glue
    /// posts TransferDone on.
    [[gnu::always_inline]] static bool isr() {
        const uint16_t s1 = S::status1();
        // STOPF is the SLAVE half's flag - a STOP seen after a START this
        // host did not issue: another master's, or the one unstick()
        // makes by hand - and ITEVTEN routes it here in every phase.
        // Left standing it re-enters this vector without end (measured:
        // the storm after an unstick). Its sequence ends in a CTLR1
        // write, which must not happen while START or STOP stands, so it
        // waits for them to leave - a bit time at most.
        if ((s1 & i2c_stopf) != 0u && !S::starting() && !S::stopping()) {
            S::clear_stopf();
        }
        switch (phase_) {
            case Phase::idle:
                // Nothing in flight: an ADDR here is the slave half's
                // too, and its sequence (STAR1 read above, then STAR2)
                // writes nothing.
                if ((s1 & i2c_addr) != 0u) {
                    (void)S::status2();
                }
                return false;

            case Phase::start_tx:
            case Phase::start_rx:
            case Phase::restart:
            case Phase::refused:
                if ((s1 & i2c_sb) == 0u) {
                    return false;
                }
                if (phase_ == Phase::refused || (phase_ == Phase::restart && (s1 & i2c_af) != 0u)) {
                    return close_refused();
                }
                // EV5: the address, with the direction bit.
                if (phase_ != Phase::start_tx) {
                    prime_receive();
                    S::data(static_cast<uint8_t>((t_.addr << 1) | 1u));
                    phase_ = Phase::addr_rx;
                } else {
                    S::data(static_cast<uint8_t>(t_.addr << 1));
                    phase_ = Phase::addr_tx;
                }
                return false;

            case Phase::addr_tx:
                if ((s1 & i2c_addr) == 0u) {
                    return false;
                }
                if (t_.tx_len == 0u) {
                    // The probe: acknowledged, and that was the question.
                    (void)S::clear_addr();
                    S::stop();
                    return finish(i2c_ok);
                }
                if constexpr (has_engines) {
                    // A PLAIN write on the transmit engine; the write half
                    // of a write-then-read stays on the pump (the class
                    // comment: the repeated START wants the TxE of its
                    // last byte).
                    if (t_.rx_len == 0u) {
                        S::dma(true, false);
                        (void)TxEngine::start(std::span<const uint8_t>(t_.tx.get(), t_.tx_len));
                        (void)S::clear_addr();
                        phase_ = Phase::tx_dma;
                        return false;
                    }
                }
                (void)S::clear_addr();
                phase_ = Phase::tx;
                S::buffer_interrupt(true);   // EV8_1 follows at once
                return false;

            case Phase::tx:
                return transmit_step(s1);

            case Phase::tx_dma:
                // BTF with the channel's count at zero: the engine wrote
                // every byte and the last one is out on the wire. BTF with
                // bytes still to move is a channel that stopped serving -
                // a live one refills the data register within a few bus
                // cycles of each TxE, a byte time ahead of BTF - and BTF
                // is a level that would re-enter this vector for ever: the
                // tenure ends i2c_dma_fault, its STOP requested (on the
                // CH32V203, a request another peripheral left held on the
                // channel's OR froze it so, docs/ch32vx03/dma.md).
                if ((s1 & i2c_btf) != 0u) {
                    if (!dma_tx_drained()) {
                        put_engines_away();
                        S::stop();
                        return finish(i2c_dma_fault);
                    }
                    dma_tx_done();
                    S::stop();   // EV8_2: a plain write's end
                    return finish(i2c_ok);
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

            case Phase::closing:
                // The void write's address acknowledged: ADDR cleared, and
                // the STOP requested behind it goes out.
                if ((s1 & i2c_addr) == 0u) {
                    return false;
                }
                (void)S::clear_addr();
                return finish(i2c_nack_data);
        }
        return false;
    }

    /// The ERROR vector's body - call from i2c1_er's handler. Every
    /// error ends the tenure with its i2c_bus.hpp code; a NACK is on
    /// the address or on a byte by the phase it landed in. True when
    /// the tenure just completed.
    [[gnu::always_inline]] static bool error_isr() {
        const uint16_t s1 = S::status1();
        const uint16_t errs = s1 & i2c_errors;
        if (errs == 0u) {
            return false;
        }
        S::clear_errors(errs);
        if (phase_ == Phase::idle) {
            return false;
        }
        uint8_t st = i2c_bus_error;
        if ((errs & i2c_af) != 0u && phase_ == Phase::restart) {
            // The LAST WRITTEN byte refused, its repeated START already
            // requested (request_restart()): the NACK is the byte's, and
            // the Sr the controller generates anyway is waited for - its
            // SB closes the tenure (close_refused(), the class comment).
            phase_ = Phase::refused;
            return false;
        }
        if ((errs & i2c_af) != 0u && phase_ == Phase::closing) {
            // The void write's address refused too: its STOP is requested.
            return finish(i2c_nack_data);
        }
        if ((errs & i2c_af) != 0u) {
            const bool on_address = phase_ == Phase::addr_tx || phase_ == Phase::addr_rx ||
                                    phase_ == Phase::start_tx || phase_ == Phase::start_rx;
            st = on_address ? i2c_nack_addr : i2c_nack_data;
            S::stop();   // 15.5.2: the host must generate the STOP
        } else if ((errs & i2c_arlo) != 0u) {
            st = i2c_arb_lost;   // the hardware released the bus already
        } else {
            S::stop();   // BERR in master mode: 15.5.1 leaves the abort to us
        }
        if constexpr (has_engines) {
            put_engines_away();
        }
        return finish(st);
    }

    /// The DMA channels' interrupt body - call from BOTH channels'
    /// vectors. The receive block completing ends a read phase (STOP
    /// after it); the transmit channel interrupts on an error alone (BTF
    /// on the event vector ends a write phase). A transfer error ends the
    /// tenure with i2c_dma_fault. INTFR is read once for both. True when
    /// the tenure just completed.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint32_t intfr = RxEngine::block_flags();
            const uint8_t tx = TxEngine::service(intfr);
            if ((tx & TxEngine::flag_error) != 0u) {
                put_engines_away();
                S::stop();
                return finish(i2c_dma_fault);
            }
            const uint8_t rx = RxEngine::service(intfr);
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
    /// Returns the pulses it took a stuck client to release SDA (0 =
    /// the wire was never stuck), or 0xFF when nine pulses and a STOP
    /// left SDA low - a short, not a client. A healthy wire is left
    /// alone.
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
        S::clear_errors(i2c_errors);
        // The STOP made by hand is seen by the instance's own slave half,
        // which raises STOPF with nothing in flight (measured); cleared
        // here, its sequence's two halves (15.10.6).
        if (S::flag(i2c_stopf)) {
            S::clear_stopf();
        }
        Pfic::enable(S::event_irq());
        Pfic::enable(S::error_irq());
        if (!free_now) {
            return 0xFF;
        }
        return released_at == 0xFF ? 0u : released_at;
    }

    /// Put the ENGINE back where start() is legal - the verb a timed
    /// I2cBus calls on a tenure that never answered: the engines put
    /// away, SWRST pulsed (the chapter's own verb for a controller
    /// stuck BUSY), the timing rewritten, PE back. The WIRE stays the
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
        S::interrupts(i2c_iterren, i2c_itevten | i2c_itbufen);
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
        // THE RESET LINE BEFORE THE GATE: on the CH32V203 and the CH32V303
        // a request DMAEN raised is held through DMAEN and PE cleared and
        // frozen on the channel's OR by the gate, stalling the next owner
        // of channel 6 or 7 - USART2's here (docs/ch32vx03/dma.md). Not
        // measured on this die (docs/ch32v00x/i2c.md); the pulse, the
        // vendor's own I2C_DeInit, leaves none either way.
        S::reset();
        S::bus_clock(false);
        SclPin::release();
        SdaPin::release();
    }

private:
    /// `restart`: the repeated START requested behind the last written
    /// byte, SB not yet seen - the phase in which a NACK is that byte's.
    /// `refused`: that NACK taken, the Sr still coming; `closing`: the
    /// void write that ends a refused tenure, its address out and its STOP
    /// requested (close_refused()).
    enum class Phase : uint8_t {
        idle, start_tx, start_rx, restart, refused, closing, addr_tx, addr_rx, tx, tx_dma, rx,
        rx_dma,
    };

    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio I2cHost: the DMA engines must carry uint8_t elements");

    /// The one exit: the status set, the phase idled, and the event line
    /// dropped with the buffer one, in one store. A STOP requested at BTF
    /// leaves BTF standing until the condition goes out on the wire
    /// (15.10.6: the hardware clears it "after initiating a start or stop
    /// event"), and with the line up that is a vector re-entered for
    /// nothing until then - measured, one entry at 400 kHz and two at
    /// 100 kHz on every write tenure.
    static bool finish(uint8_t st) {
        status_ = st;
        phase_ = Phase::idle;
        S::interrupts(0, i2c_itevten | i2c_itbufen);
        return true;
    }

    /// EV8 on the buffer vector, and EV8_2 on the event vector. A plain
    /// write drops the buffer vector with its last byte and ends on BTF; a
    /// read half to follow keeps it, and the TxE that says the last byte
    /// went into the shifter - or one served later, with BTF up - requests
    /// the repeated START (the class comment).
    [[gnu::always_inline]] static bool transmit_step(uint16_t s1) {
        if ((s1 & i2c_txe) == 0u) {
            return false;   // BTF implies TxE
        }
        if (pos_ < t_.tx_len) {
            S::data(t_.tx.get()[pos_]);
            ++pos_;
            if (pos_ >= t_.tx_len && t_.rx_len == 0u) {
                S::buffer_interrupt(false);   // BTF carries the end
            }
            return false;
        }
        if (t_.rx_len != 0u) {
            request_restart();
            return false;
        }
        if ((s1 & i2c_btf) != 0u) {
            S::stop();   // EV8_2
            return finish(i2c_ok);
        }
        return false;
    }

    /// The repeated START of a write-then-read, requested while the last
    /// written byte is still in the shifter: the controller generates it at
    /// the end of that byte (15.10.1's START bit). The buffer vector goes
    /// down first, the TxE that brought this here having no more to say.
    [[gnu::always_inline]] static void request_restart() {
        S::interrupts(0, i2c_itbufen);
        phase_ = Phase::restart;
        S::start();
    }

    /// EV5 of the Sr behind a refused last byte: the address with the
    /// WRITE bit, which clears SB, and STOP requested behind it - "after the
    /// current byte transfer" (15.10.1) - so the wire carries a void write
    /// and the controller is left as a plain write leaves it. AF is cleared
    /// here too, for the case in which this vector saw it first.
    [[gnu::always_inline]] static bool close_refused() {
        S::clear_errors(i2c_af);
        S::data(static_cast<uint8_t>(t_.addr << 1));
        S::stop();
        phase_ = Phase::closing;
        return false;
    }

    /// The ACK/POS arrangement the read phase's length wants, set
    /// BEFORE the address goes out (15.3's receive procedures).
    static void prime_receive() {
        if (t_.rx_len == 2u && !dma_serves_rx()) {
            S::pos(true);
            S::ack(true);
        } else {
            S::ack(t_.rx_len != 1u);
        }
    }

    /// EV6 on a read: the procedure by count.
    static bool begin_receive() {
        if constexpr (has_engines) {
            if (dma_serves_rx()) {
                // DMAEN with LAST - the block's last byte is NACKed - and
                // THE EVENT LINE DOWN, in one store: dma_isr() ends the
                // phase and the event vector has nothing to serve in it,
                // while over a channel that does not serve its request RxNE
                // and BTF would stand and re-enter it for ever, the thread
                // and the bus's timeout starved. The error line stays.
                S::interrupts(i2c_dmaen | i2c_last, i2c_itevten);
                (void)RxEngine::start(std::span<uint8_t>(t_.rx.get(), t_.rx_len));
                (void)S::clear_addr();
                phase_ = Phase::rx_dma;
                return false;
            }
        }
        pos_ = 0;
        if (t_.rx_len == 1u) {
            // ACK is already clear (prime_receive): clear ADDR, then STOP
            // at once; the byte lands on RxNE.
            (void)S::clear_addr();
            S::stop();
            phase_ = Phase::rx;
            S::buffer_interrupt(true);
            return false;
        }
        if (t_.rx_len == 2u) {
            // POS and ACK were set: clear ADDR, then ACK off - the NACK
            // lands on the second byte; one BTF delivers both.
            (void)S::clear_addr();
            S::ack(false);
            phase_ = Phase::rx;
            return false;
        }
        (void)S::clear_addr();
        phase_ = Phase::rx;
        S::buffer_interrupt(t_.rx_len > 3u);   // RxNE while more than three remain
        return false;
    }

    /// The receive pump, per the count's procedure.
    [[gnu::always_inline]] static bool receive_step(uint16_t s1) {
        uint8_t* out = t_.rx.get();
        if (t_.rx_len == 1u) {
            if ((s1 & i2c_rxne) == 0u) {
                return false;
            }
            out[0] = S::data();
            return finish(i2c_ok);
        }
        if (t_.rx_len == 2u) {
            if ((s1 & i2c_btf) == 0u) {
                return false;
            }
            S::stop();
            out[0] = S::data();
            out[1] = S::data();
            S::pos(false);
            return finish(i2c_ok);
        }
        const uint8_t remaining = static_cast<uint8_t>(t_.rx_len - pos_);
        if (remaining > 3u) {
            if ((s1 & i2c_rxne) != 0u) {
                out[pos_] = S::data();
                ++pos_;
                if (t_.rx_len - pos_ == 3u) {
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
            return t_.rx_len >= 2u;
        } else {
            return false;
        }
    }
    /// The transmit block has moved every byte: its count read at zero.
    static bool dma_tx_drained() {
        if constexpr (has_engines) {
            return TxEngine::progress().remaining == 0u;
        } else {
            return true;
        }
    }
    /// The write block's end, proved by BTF: the engine released and the
    /// controller's requests dropped.
    static void dma_tx_done() {
        if constexpr (has_engines) {
            (void)TxEngine::complete();
            S::dma(false, false);
        }
    }
    static void put_engines_away() {
        if constexpr (has_engines) {
            (void)TxEngine::abandon();
            RxEngine::stop();   // the binding stands: the next start() needs no arm()
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
    /// wire. A STOP takes a bit period; what can hold it longer is a
    /// slow client stretching the clock after the last acknowledge while
    /// it digests a write, and five milliseconds - a fifth of SMBus's
    /// 25 ms clock-low limit - is the bound. What start() spends here is
    /// the kernel dispatch's time, which is why it is bounded at all.
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

    /// The tenure's own copy of the Request's fields its entries read.
    struct Tenure {
        Borrowed<const uint8_t, Lease::reply> tx;
        Borrowed<uint8_t, Lease::reply> rx;
        uint8_t addr;
        uint8_t tx_len;
        uint8_t rx_len;
    };
    static inline Tenure t_{};
    static inline uint8_t pos_ = 0;
    static inline volatile Phase phase_ = Phase::idle;
    static inline uint8_t status_ = i2c_ok;
    static inline I2cSpeed applied_ = I2cSpeed::standard_100k;
    static inline I2cDuty duty_ = I2cDuty::ratio_2;
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

/**
 * I2cClient<n, pins>
 *
 * The target side. The peripheral matches the own address(es), ACKs it,
 * raises ADDR and STRETCHES SCL until the sequence that clears it runs
 * (15.4); every byte then comes on RxNE or is asked for on TxE, with
 * the clock stretched while the flag stands unless NOSTRETCH. A read
 * tenure ends with the host's NACK (AF, on the error vector); a write
 * tenure with STOPF.
 *
 *   extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() { act(Peer::service()); }
 *   extern "C" BRIO_CH32_INTERRUPT void i2c1_er_handler() { act(Peer::error_service()); }
 */
template <uint8_t n, I2cPins pins = i2c1_default_pins>
class I2cClient {
    using S = I2c<n>;

    static_assert(i2c_pins_valid(pins),
                  "brio I2cClient: these I2C pads are not a bus - SCL and SDA must both "
                  "be named and distinct");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;

public:
    I2cClient() = delete;

    using Resource = S;
    static constexpr I2cPins pin_pads = pins;

    /// Bring the instance up as a bus target. FREQ must still state the
    /// bus clock (the peripheral's own timing needs it even as a
    /// target), so the same 8 MHz floor applies. `no_stretch` gives up
    /// the clock stretch, and with it the guarantee against OVR.
    template <typename Clock>
    static bool init(Clock clock, const I2cAddressConfig& addr, bool no_stretch = false) {
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        S::bus_clock(true);
        S::reset();
        Afio::remap_i2c1(pins.remap);
        const auto t = i2c_timing_for(clock_hz(clock), I2cSpeed::standard_100k);
        if (!t) {
            return false;
        }
        S::timing(*t);
        if (!S::addresses(addr)) {
            return false;
        }
        S::no_stretch(no_stretch);
        S::enable();
        S::ack(true);   // answer the own address
        SclPin::function(PinDrive::open_drain);
        SdaPin::function(PinDrive::open_drain);
        S::event_interrupt(true);
        S::buffer_interrupt(true);
        S::error_interrupt(true);
        Pfic::enable(S::event_irq());
        Pfic::enable(S::error_irq());
        return true;
    }

    // ---- the protocol surface, polled -----------------------------------------

    /// An address match is pending (SCL held meanwhile, unless
    /// NOSTRETCH).
    static bool addressed() { return S::flag(i2c_addr); }
    /// Answer the match and release the clock: EV1's STAR1-then-STAR2
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
    /// The host refused a byte this client gave: the end of a read
    /// tenure (15.5.2: the hardware released the bus).
    static bool host_nacked() { return S::flag(i2c_af); }
    static void clear_nack() { S::clear_errors(i2c_af); }
    static bool overrun() { return S::flag(i2c_ovr); }
    static void clear_overrun() { S::clear_errors(i2c_ovr); }
    static uint16_t flags() { return S::status1(); }

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
        const uint16_t errs = s1 & i2c_errors;
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

// At 48 MHz: standard mode CCR = 240 (48e6 / 200e3), fast mode CCR = 40
// at DUTY 2 (48e6 / 1.2e6) and 5 at 16/9 (48e6 / 10e6 rounded up).
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k)->ckcfgr == 240u);
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k)->freq_mhz == 48u);
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k)->ckcfgr == (i2c_fs | 40u));
static_assert(i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k, I2cDuty::ratio_16_9)->ckcfgr ==
              (i2c_fs | i2c_duty | 5u));
static_assert(i2c_scl_hz(48'000'000UL, *i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k)) == 100'000UL);
static_assert(i2c_scl_hz(48'000'000UL, *i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k)) == 400'000UL);
// Rounded UP: at 24 MHz fast mode wants 20; at 8 MHz, 6.67 -> 7 (381 kHz).
static_assert(i2c_timing_for(8'000'000UL, I2cSpeed::fast_400k)->ckcfgr == (i2c_fs | 7u));
static_assert(i2c_scl_hz(8'000'000UL, *i2c_timing_for(8'000'000UL, I2cSpeed::fast_400k)) == 380'952UL);
// The window: below 8 MHz nothing is legal (15.10.2).
static_assert(!i2c_timing_for(6'000'000UL, I2cSpeed::standard_100k).has_value());
static_assert(!i2c_timing_for(3'000'000UL, I2cSpeed::fast_400k).has_value());
static_assert(i2c_timing_for(8'000'000UL, I2cSpeed::standard_100k)->ckcfgr == 40u);

static_assert(i2c_pins_valid(i2c1_default_pins) && i2c_pins_valid(i2c1_pins_for(1)));
// Each part's remap table (afio.hpp): code 1 is PD1/PD0 on both.
static_assert(i2c1_pins_for(1).scl == Pad{'D', 1} && i2c1_pins_for(1).sda == Pad{'D', 0});
static_assert(device::part == Ch32Part::v003 ? i2c1_pins_for(2).sda == Pad{'C', 6}
                                             : i2c1_pins_for(5).sda == Pad{'D', 1});
static_assert(!i2c_pins_valid(I2cPins{.scl = {'C', 2}, .sda = {'C', 2}}));
static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x48}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x80}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x123, .ten_bit = true, .second = 0x10}));
static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x123, .ten_bit = true}));

} // namespace brio
