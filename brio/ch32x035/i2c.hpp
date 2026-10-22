/*
 * i2c.hpp
 *
 * The I2C of the CH32X035 (RM V1.8 ch. 15) as a bus for a program: a
 * sensor or a memory read and written from an active object through
 * the arbiter of util/i2c_bus.hpp, or this chip as a target another
 * controller addresses. One instance, host and client, 7- and 10-bit
 * addresses, dual addressing and the general call, two speeds with fast
 * mode's two duty shapes, clock stretching, PEC, two DMA requests - the
 * STM32F1's I2C under WCH's register names, an event machine (SB, ADDR,
 * BTF, ADD10, STOPF, RxNE, TxE and five errors) over two vectors, events
 * on table 7-1's entry 30 and errors on 31. Three layers, the shape every
 * bus driver of brio has:
 *
 *  - `I2c<n>` is the resource: the register block, its gate and reset,
 *    the column, the clock arithmetic, the flags and the sequences that
 *    clear them, the control bits as verbs. It decides nothing.
 *  - `I2cHost<n, pins, TxEngine, RxEngine, Placement>` is the engine util/
  *    i2c_bus.hpp's I2cBus drives: the Request is the same struct on every
 *    stratum, verbatim - {addr, tx span, rx span, reply, speed}, one bus tenure
 *    that is a write, a read, a write-then-read with a repeated START, or
 *    the empty probe - and the outcome vocabulary is i2c_bus.hpp's.
 *  - `I2cClient<n, pins>` is the target side: a polled surface and two
 *    interrupt bodies, deliberately thin, because a client is a protocol
 *    and which protocol is the application's.
 *
 * The smallest use, a register read from a sensor, is written out at
 * I2cHost below: `using Bus = brio::I2cHost<1>` over the part's default
 * column, `I2cBus<Bus, P, ...>` as the arbiter in the program's Tenuto,
 * `Bus::init(clock)` after the clock task, the two vectors bound to
 * isr() and error_isr() under BRIO_CH32_INTERRUPT, then a Request posted
 * to the arbiter with its buffers lent until the reply.
 *
 * Who calls what, and from where. init(), release(), unstick() and
 * recover() are setup and recovery verbs, main context (the arbiter
 * calls recover() from its own dispatch on a timed-out tenure); init()
 * waits for nothing beyond the gate. start() is called by the arbiter
 * from its dispatch, main context, and waits, bounded, for the last
 * tenure's STOP to leave CTLR1 (5 ms at most) and for BUSY (100 us):
 * what it spends is the dispatch's time. isr(), error_isr() and
 * dma_isr() are the three vectors' bodies: the program's handlers call
 * them and nothing else does; each runs to completion, masks nothing,
 * and the first ends a tenure with a bounded poll of TxE (a few STAR1
 * reads) and, on the last two bytes of a read, a bounded wait for RxNE.
 * The Request's two buffers are lent from the post to the I2cDone the
 * arbiter replies with: untouched and in scope until then, which nothing
 * checks. The client's polled verbs are the loop's, its service() and
 * error_service() the vectors' bodies. Nothing here masks interrupts.
 *
 * The block may not be there. The datasheet (V1.7, table 2-1's note 3 and
 * the note of 1.4.16) withdraws the I2C from every lot "with a 0 in the
 * penultimate 5 digits of the lot number", and no register names a die's
 * lot. So `I2c<n>::present()` asks the block itself - a CKCFGR value
 * written with PE clear and read back - and both `init()`s refuse a block
 * that does not hold what they wrote into it (docs/ch32x035/i2c.md has
 * the bench die's answer).
 *
  * The machine this chapter prescribes is a set of software sequences,
 * each stretching SCL low until it completes (15.3's notes to figures 15-3
 * and 15-4): EVT5 (SB up: read STAR1, write the address), EVT6 (ADDR up:
 * read STAR1 then STAR2), EVT8_1 and EVT8 (TxE: write DATAR), EVT8_2 (TxE
 * and BTF: the last byte is out, STOP), EVT7 (RxNE: read DATAR) - and a
 * receive procedure that depends on the count: one byte wants ACK cleared
 * before ADDR is cleared and STOP set right after (EVT6_1); two bytes want
 * POS set with ACK (15.9.1's note on POS), ACK cleared after ADDR, and the
 * pair read after one BTF; three or more run on RxNE until three remain,
 * then wait for BTF twice - the first clears ACK and reads N-2, the
 * second sets STOP and reads N-1 and N. All of it is in `isr()`, phase by
 * phase, and the reason ITBUFEN is switched on and off during a tenure:
 * TxE and RxNE interrupt only while the byte pump needs them, BTF (under
 * ITEVTEN alone, 15.9.2) carries the rest. ITEVTEN itself is the tenure's:
 * BTF stands until the STOP requested on it is on the wire (15.9.6: the
 * hardware clears it "after initiating a start or stop event"), and the
 * client half's STOPF and ADDR would enter the vector between tenures, so
 * the event line is raised by start() and dropped at the completion in
 * the same CTLR2 store as the buffer line and the DMA requests
 * (`control2()`). A DMA read phase takes it down too: the receive
 * channel's completion ends that phase.
 *
 * The entry that clears ADDR loads the first two bytes itself. EVT8_1 is
 * "TxE=1, shift register empty, data register empty, write DR" and EVT8
 * follows it as the first byte moves into the shifter (figure 15-3), so
 * the data register and the shifter take two bytes in that one entry -
 * TxE polled for a bounded count of STAR1 reads, the buffer line raised
 * if it does not come - and a one- or two-byte write, or a register index
 * before a read, takes no TxE interrupt at all.
 *
 * The repeated START of a write-then-read is requested while the last
 * written byte is still going out, on the TxE that says it went into the
 * shifter: 15.9.1's START bit, in master mode, is "repeated start
 * generation", and set while a byte shifts it goes out as that byte ends.
 * It is not the chapter's own end of a write - figure 15-3 ends one at
 * EVT8_2, with SCL held low past the last acknowledge - and it is the
 * order the far end needs when it is a POLLED target of this block's
 * lineage: such a target loses the last written byte when the START comes
 * after BTF (measured on the CH32V203C8T6 and the CH32V303VCT6,
 * docs/ch32vx03/i2c.md), and a register index followed by a read is that
 * shape. The desk's partner is one (the CH32V006's twi_peer), and in this
 * order it took every written byte, one to four before a read, at both
 * speeds, pump and engines (measured, docs/ch32x035/i2c.md). The price is in the
 * error path: the START is requested before the last byte's acknowledge
 * is known, so the phase between the request and SB is its own
 * (`restart`) and a NACK in it is the byte's - i2c_nack_data. The
 * repeated START is not withdrawn: the controller generates it after the
 * refused byte all the same, and a CTLR1 store that clears START and sets
 * STOP races that generation (lost at 400 kHz on the CH32V006 and with
 * the core at 8 MHz on the CH32V203, the bus left held). So the refusal
 * lets the Sr out and closes the tenure from its SB with the one sequence
 * EVT5 allows: the address with the WRITE bit and STOP requested behind
 * it - a void write, Sr A P on the wire - its ADDR cleared or its own
 * NACK taken before the tenure reports i2c_nack_data.
 *
 * There is no rise-time register. Table 15-1 ends at CKCFGR (0x1C) and so
 * does the chapter's prose: the SCL timing is CTLR2.FREQ and CKCFGR (CCR
 * under F/S and DUTY), and FREQ "must be set between 000100b and
 * 111100b", 4 to 60 MHz (15.9.2). HCLK runs to 48 MHz on this series and
 * every bus runs at HCLK, so the window's ceiling is never reached and
 * its floor is: an HCLK under 4 MHz - 3 MHz and below on the HPRE ladder -
 * has no legal timing and init() refuses it. The CCR arithmetic is the
 * F1's, stated by WCH's own I2C_Init (the register description gives
 * none): standard mode divides by 2 x CCR, fast mode by 3 x CCR at DUTY 2
 * and 25 x CCR at DUTY 16/9 - rounded up here so a bus never runs faster
 * than asked, where the vendor's library rounds down.
 *
 * The DMA requests are channels 6 (transmit) and 7 (receive), figure 9-1
 * (ch32x035/dma_engine.hpp's table): the channel is the request, so an
 * engine slot on any other channel is refused at compile time. Channel 6
 * is also USART2's receive request and channel 7 its transmit - an OR of
 * the two peripherals' requests on each - which is why release() pulses
 * the block's reset line (docs/design/dma.md's release contract). With
 * engines a plain write of three bytes or more runs on the transmit
 * engine (two ride EVT8_1's entry) and ends on BTF, which the event line
 * carries anyway, with the channel's count read at zero: the transmit
 * engine is armed for its errors alone and a write takes no DMA
 * interrupt. A read phase of two bytes or more runs on the receive engine
 * under CTLR2.LAST, which makes the controller NACK the last byte a block
 * takes (15.9.2), and ends on the block's completion. A one-byte read
 * stays on the pump (EVT6_1's ACK-before-ADDR has no DMA form), and so
 * does the write half of a write-then-read: its repeated START wants the
 * TxE of the last byte, which a block armed for its errors alone does not
 * report.
 *
 * BUSY is the wire (15.9.7: "low level present in SDA or SCL"), still
 * updated with PE clear, and a tenure that ends without the peripheral
 * seeing its STOP can leave it standing over an idle wire - 15.9.1's own
 * case for SWRST, "when no stop condition is detected on the bus but the
 * busy bit is 1". `start()` answers it when its bounded wait for BUSY
 * runs out and both lines read high, the wire being the guard that keeps
 * another master's busy bus out of it, and counts it (`unwedges()`).
 *
 * A START into a held line is not refused by this die (measured,
 * docs/ch32x035/i2c.md): set by a block that saw another device take SDA,
 * it raises SB some 20 us later with the line still held, and the address
 * written behind it loses arbitration at its first one bit - so a tenure
 * into a stuck target comes back i2c_arb_lost within a byte, through
 * error_isr(); set by a block brought up over a line already held, it
 * stays pending with BUSY standing, and the per-bus timeout answers. And
 * the client half raises STOPF on a STOP it was not addressed in, ACK
 * clear or set, which isr() and unstick() clear by EVT4.
 *
 * The pads are a column (afio.hpp, I2C1_RM[2:0]): six columns, PA10/PA11,
 * PA13/PA14, the USB pads PC16/PC17 in both orders and the debug port's
 * PC18/PC19 in both orders. A pad under I2C1's alternate function is open
 * drain by itself (8.2.8, table 8-5): this series has no open-drain GPIO
 * at all, and the pull-ups are the wire's. Three refusals come from the
 * package and the board. A column whose pads this part does not bond, or
 * whose pads share a package pin with another (the datasheet's notes 4 to
 * 7 forbid such a pad as an output - PC16 and PC17 on every package but
 * the QFN20), does not compile. A column on the debug port's pads is
 * refused by init() while the two-wire port is still the probe's, as the
 * USARTs' are. And a column on PC16/PC17 is refused while AFIO_CTLR's
 * USB_IOEN gives those pads to the USB transceiver (8.3.2.4). Which
 * column a part has outside the debug port is its own: the CH32X035R8,
 * C8 and the CH32X033F8 have PA10/PA11, the CH32X035F8 PC16/PC17 alone,
 * and the F7 and the two 28-pin parts none - their default column is the
 * debug port's (`i2c1_default_pins`). AFIO_CTLR's pull-up modes for PC16
 * and PC17 (8.3.2.4) do not act under the alternate function (measured):
 * the wire's resistors are the only pull-ups an I2C on those pads has.
 *
 * What the register description names and the bits do not have. CTLR1
 * carries ENARP (bit 4) and its STOP bit is "set by hardware when a
 * timeout error is detected", ITERREN lists TIMEOUT and SMBAlert, and the
 * datasheet's feature list says "support SMBus/PMBus" - but CTLR1 has no
 * SMBUS or SMBTYPE bit (3:1 reserved) and STAR1 no TIMEOUT or SMBALERT
 * (15:13 reserved): the SMBus mode is declined, and ENARP is a verb of the
 * resource with no engine behind it (docs/ch32x035/vendor/README.md).
 *
 * The event pump is one function of its placement, the host's last
 * template parameter (util/placement.hpp): its vector calls the
 * placement's trampoline, the entry's whole path flattened into it - in
 * the flash by default, in SRAM under InRam, where an entry takes a
 * fraction of what it takes behind the flash's two wait states at 48 MHz
 * (RM 20.3.1). The wire does not move either way (SCL stretches while an
 * entry runs); the core's share of a tenure does, and so does the longest
 * handler the image's other hosts wait behind (docs/ch32x035/i2c.md).
 *
 * Not covered yet: 10-bit addressing on the host side (the resource has
 * the mode and the client matches such an address; EVT9 has no user -
 * i2c_bus.hpp's Request carries a 7-bit address on every stratum); PEC as
 * a tenure shape; a wake from a sleep on an address match, which this
 * stratum's power chapter does not offer.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <span>
#include <type_traits>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/device.hpp"
#include "ch32x035/dma_engine.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "ch32x035/platform.hpp"
#include "util/clock.hpp"
#include "util/i2c_bus.hpp"
#include "util/placement.hpp"

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
    volatile uint16_t CKCFGR;   ///< 0x1c - the last register of the block
    uint16_t RESERVED7;
};

/// I2C1 on PB1 at 0x40005400 (table 15-1); the series has no second.
inline constexpr uint32_t i2c1_base = pb1_base + 0x5400;

constexpr uint32_t i2c_base_for(uint8_t n) { return n == 1 ? i2c1_base : 0; }

// CTLR1 (15.9.1)
inline constexpr uint16_t i2c_pe        = 1u << 0;
inline constexpr uint16_t i2c_enarp     = 1u << 4;
inline constexpr uint16_t i2c_enpec     = 1u << 5;
inline constexpr uint16_t i2c_engc      = 1u << 6;
inline constexpr uint16_t i2c_nostretch = 1u << 7;
inline constexpr uint16_t i2c_start     = 1u << 8;
inline constexpr uint16_t i2c_stop      = 1u << 9;
inline constexpr uint16_t i2c_ack       = 1u << 10;
inline constexpr uint16_t i2c_pos       = 1u << 11;
inline constexpr uint16_t i2c_pec       = 1u << 12;
inline constexpr uint16_t i2c_swrst     = 1u << 15;
// CTLR2 (15.9.2)
inline constexpr uint16_t i2c_freq_mask = 0x3Fu;
inline constexpr uint16_t i2c_iterren   = 1u << 8;
inline constexpr uint16_t i2c_itevten   = 1u << 9;
inline constexpr uint16_t i2c_itbufen   = 1u << 10;
inline constexpr uint16_t i2c_dmaen     = 1u << 11;
inline constexpr uint16_t i2c_last      = 1u << 12;
// OADDR1 (15.9.3): 14:10 reserved - no "bit 14 kept at one" here.
inline constexpr uint16_t i2c_addmode   = 1u << 15;
// OADDR2 (15.9.4)
inline constexpr uint16_t i2c_endual    = 1u << 0;
// STAR1 (15.9.6)
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
/// The five errors STAR1 has - all of ITERREN's sources the register
/// description gives a bit (the header: no TIMEOUT, no SMBALERT).
inline constexpr uint16_t i2c_errors    = i2c_berr | i2c_arlo | i2c_af | i2c_ovr | i2c_pecerr;
// STAR2 (15.9.7)
inline constexpr uint16_t i2c_msl       = 1u << 0;
inline constexpr uint16_t i2c_busy      = 1u << 1;
inline constexpr uint16_t i2c_tra       = 1u << 2;
inline constexpr uint16_t i2c_gencall   = 1u << 4;
inline constexpr uint16_t i2c_dualf     = 1u << 7;
inline constexpr uint16_t i2c_pec_shift = 8;
// CKCFGR (15.9.8)
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

/// What CTLR2.FREQ and CKCFGR hold for one speed at one bus clock.
struct I2cTiming {
    uint8_t freq_mhz = 0;    ///< CTLR2.FREQ, the bus clock in whole MHz
    uint16_t ckcfgr = 0;     ///< CCR with F/S and DUTY
};

/// 15.9.2's window for FREQ: 000100b..111100b.
inline constexpr uint32_t i2c_min_hz = 4'000'000UL;
inline constexpr uint32_t i2c_max_hz = 60'000'000UL;

/**
 * The F1 arithmetic (the file header): standard mode SCL = pclk /
 * (2 x CCR), fast mode pclk / (3 x CCR) at DUTY 2 and pclk / (25 x CCR)
 * at DUTY 16/9. CCR is rounded up - a bus at most as fast as asked - with
 * the floors WCH's I2C_Init keeps (4 in standard mode, 1 in fast). FREQ is
 * the clock's whole megahertz, truncated as that function truncates it.
 * Nullopt outside FREQ's window, or when CCR would not fit its twelve
 * bits.
 */
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

/// Which pads carry the two lines, and the remap code that puts the
/// peripheral on them (afio.hpp's I2C1 columns): init() writes the code.
struct I2cPins {
    Pad scl{};
    Pad sda{};
    uint8_t remap = 0;
};

/// The pins of column `code`.
constexpr I2cPins i2c1_pins_for(uint8_t code) {
    const I2cPadSet p = afio_i2c1_pads(code);
    return I2cPins{.scl = p.scl, .sda = p.sda, .remap = code};
}

/**
 * Is that column a bus on this part? Both lines named and distinct, the
 * code one of the field's, and both pads ones the package brings out
 * alone on their pins - a pad that shares its package pin with another is
 * forbidden as an output by the datasheet (notes 4 to 7), and an I2C pad
 * is an alternate output.
 */
constexpr bool i2c_pins_valid(const I2cPins& p) {
    return p.scl.valid() && p.sda.valid() && !(p.scl == p.sda) && p.remap < afio_i2c1_codes &&
           afio_i2c1_pads(p.remap).scl == p.scl && afio_i2c1_pads(p.remap).sda == p.sda &&
           pad_can_drive(p.scl) && pad_can_drive(p.sda);
}

/// Does the column put a line on the debug port's pads (device.hpp)?
constexpr bool i2c_pins_on_debug_port(const I2cPins& p) {
    const Pad dio{debug_swdio_port, debug_swdio_pin};
    const Pad clk{debug_swclk_port, debug_swclk_pin};
    return p.scl == dio || p.scl == clk || p.sda == dio || p.sda == clk;
}

/// Does the column use the USB pads PC16/PC17 (AFIO_CTLR's USB_IOEN)?
constexpr bool i2c_pins_on_usb_pads(const I2cPins& p) {
    const auto usb = [](Pad x) { return x.port == 'C' && (x.pin == 16u || x.pin == 17u); };
    return usb(p.scl) || usb(p.sda);
}

/// The lowest code that is a bus on this part off the debug port, or -
/// on a part with none - the lowest that is a bus at all (the debug
/// port's, which init() refuses while the probe owns it); 0 when there is
/// no column at all, which no part of the series is.
constexpr uint8_t i2c1_first_column() {
    for (uint8_t c = 0; c < afio_i2c1_codes; ++c) {
        if (i2c_pins_valid(i2c1_pins_for(c)) && !i2c_pins_on_debug_port(i2c1_pins_for(c))) {
            return c;
        }
    }
    for (uint8_t c = 0; c < afio_i2c1_codes; ++c) {
        if (i2c_pins_valid(i2c1_pins_for(c))) {
            return c;
        }
    }
    return 0;
}

/// The default for `I2cHost<1>` and `I2cClient<1>`: the part's first
/// column (the file header) - PC16/PC17 on the CH32X035F8.
inline constexpr I2cPins i2c1_default_pins = i2c1_pins_for(i2c1_first_column());

/// A target's addresses (15.4): one 7- or 10-bit own address, an
/// optional second 7-bit one (ENDUAL), the general call.
struct I2cAddressConfig {
    uint16_t own = 0;                  ///< 7 bits, or 10 with ten_bit
    bool ten_bit = false;
    std::optional<uint8_t> second{};   ///< OADDR2: a second 7-bit address
    bool general_call = false;
};

constexpr bool i2c_address_config_valid(const I2cAddressConfig& a) {
    if (a.ten_bit) {
        return a.own <= 0x3FFu && !a.second;   // ENDUAL is a 7-bit arrangement (15.4)
    }
    return a.own <= 0x7Fu && (!a.second || *a.second <= 0x7Fu);
}

// =============================================================================
// The resource
// =============================================================================

/**
 * I2c<n>: the register block and its verbs. Every verb is one of the
 * chapter's sequences or one bit; the phases are the engine's.
 *
 *   using Bus = brio::I2c<1>;
 *   Bus::bus_clock(true);
 *   Bus::reset();
 *   Bus::timing(*brio::i2c_timing_for(48'000'000, brio::I2cSpeed::fast_400k));
 *   Bus::enable();
 */
template <uint8_t n>
struct I2c {
    static_assert(i2c_base_for(n) != 0, "brio I2c: this series has I2C1 alone (RM ch. 15)");

    I2c() = delete;

    static constexpr uint8_t number = n;
    /// Figure 9-1: the two channels this instance's requests reach.
    static constexpr uint8_t dma_tx_channel = dma_request_channel(DmaRequest::i2c1_tx);
    static constexpr uint8_t dma_rx_channel = dma_request_channel(DmaRequest::i2c1_rx);

    static I2cRegs& regs() { return *reinterpret_cast<I2cRegs*>(i2c_base_for(n)); }
    static constexpr Irq event_irq() { return Irq::i2c1_ev; }
    static constexpr Irq error_irq() { return Irq::i2c1_er; }
    static volatile void* data_address() { return &regs().DATAR; }

    /// The gate (RCC_APB1PCENR bit 21). Every bus runs at HCLK, so the
    /// gate is the instance's whole clock.
    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(Bus::pb1, rcc_pb1_i2c1);
        } else {
            Rcc::disable(Bus::pb1, rcc_pb1_i2c1);
        }
    }
    /// The RCC reset pulse (RCC_APB1PRSTR bit 21): every register back to
    /// its reset value, a raised DMA request withdrawn.
    static void reset() { (void)Rcc::reset(Bus::pb1, rcc_pb1_i2c1); }

    /// Put the column into AFIO's I2C1_RM. False, and nothing written, for
    /// a code this part has no pad of (afio.hpp).
    static bool remap(uint8_t code) { return Afio::remap(Remap::i2c1, code); }

    /// The timing, with PE clear on the way in: FREQ and CKCFGR are
    /// written with the peripheral disabled (15.3's order of steps).
    static void timing(I2cTiming t) {
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~i2c_pe);
        regs().CTLR2 =
            static_cast<uint16_t>((regs().CTLR2 & ~i2c_freq_mask) | (t.freq_mhz & i2c_freq_mask));
        regs().CKCFGR = t.ckcfgr;
    }

    /**
     * Is the block there? The lot note's question (the file header), put
     * to the registers: the gate opened, the block reset, a CKCFGR value
     * no reset leaves written with PE clear and read back, and the block
     * reset again. True when the register held it.
     */
    static bool present() {
        bus_clock(true);
        reset();
        regs().CKCFGR = static_cast<uint16_t>(i2c_fs | 0x0A5u);
        const bool held = regs().CKCFGR == static_cast<uint16_t>(i2c_fs | 0x0A5u);
        reset();
        return held;
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

    /// SWRST: the state machines and the flags back to reset - 15.9.1's
    /// verb for a bus that shows BUSY with no STOP in sight. Pulsed: set,
    /// then cleared.
    static void software_reset() {
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | i2c_swrst);
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~i2c_swrst);
    }

    // ---- the control bits ----------------------------------------------------

    [[gnu::always_inline]] static void start() { ctlr1(i2c_start, true); }
    [[gnu::always_inline]] static bool starting() { return (regs().CTLR1 & i2c_start) != 0u; }
    [[gnu::always_inline]] static void stop() { ctlr1(i2c_stop, true); }
    [[gnu::always_inline]] static bool stopping() { return (regs().CTLR1 & i2c_stop) != 0u; }
    [[gnu::always_inline]] static void ack(bool on) { ctlr1(i2c_ack, on); }
    /// POS: the ACK bit governs the NEXT byte to arrive rather than the
    /// current one - the two-byte receive's arrangement (15.9.1).
    [[gnu::always_inline]] static void pos(bool on) { ctlr1(i2c_pos, on); }
    static void general_call(bool on) { ctlr1(i2c_engc, on); }
    static void no_stretch(bool on) { ctlr1(i2c_nostretch, on); }
    /// ENPEC: the CRC-8 over every byte of the tenure, address included
    /// (15.8).
    static void pec(bool on) { ctlr1(i2c_enpec, on); }
    /// PEC: the byte under way IS the checksum (transmit) or is compared
    /// against it (receive); hardware clears it at the transfer's end.
    static void pec_next(bool on) { ctlr1(i2c_pec, on); }
    /// ENARP, as the register description names it (the file header: no
    /// SMBus mode bit beside it).
    static void arp(bool on) { ctlr1(i2c_enarp, on); }
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

    [[gnu::always_inline]] static void data(uint8_t v) { regs().DATAR = v; }
    [[gnu::always_inline]] static uint8_t data() { return static_cast<uint8_t>(regs().DATAR); }
    [[gnu::always_inline]] static uint16_t status1() { return regs().STAR1; }
    [[gnu::always_inline]] static uint16_t status2() { return regs().STAR2; }
    [[gnu::always_inline]] static bool flag(uint16_t mask) { return (regs().STAR1 & mask) != 0u; }
    [[gnu::always_inline]] static bool busy() { return (regs().STAR2 & i2c_busy) != 0u; }
    static bool host() { return (regs().STAR2 & i2c_msl) != 0u; }
    /// TRA: this side is transmitting in the tenure under way.
    static bool transmitting() { return (regs().STAR2 & i2c_tra) != 0u; }
    static bool second_address_matched() { return (regs().STAR2 & i2c_dualf) != 0u; }
    static bool general_call_matched() { return (regs().STAR2 & i2c_gencall) != 0u; }
    /// STAR2's upper half: the running PEC, valid while ENPEC is set.
    static uint8_t pec_value() { return static_cast<uint8_t>(regs().STAR2 >> i2c_pec_shift); }

    /// EVT6/EVT1: ADDR is cleared by reading STAR1 then STAR2, in that
    /// order. Returns STAR2, which tells the direction (TRA) and which
    /// address matched.
    [[gnu::always_inline]] static uint16_t clear_addr() {
        (void)regs().STAR1;
        return regs().STAR2;
    }
    /// EVT4: STOPF is cleared by reading STAR1 then writing CTLR1.
    static void clear_stopf() {
        (void)regs().STAR1;
        regs().CTLR1 = regs().CTLR1;
    }
    /// The five error flags are write-zero-to-clear (RW0, 15.9.6).
    static void clear_errors(uint16_t mask) {
        regs().STAR1 = static_cast<uint16_t>(~(mask & i2c_errors));
    }

    // ---- interrupts ----------------------------------------------------------

    static void event_interrupt(bool on) { ctlr2(i2c_itevten, on); }
    /// ITBUFEN: TxE and RxNE reach the event vector too.
    static void buffer_interrupt(bool on) { ctlr2(i2c_itbufen, on); }
    static void error_interrupt(bool on) { ctlr2(i2c_iterren, on); }

    /// CTLR2 in one store: FREQ as the timing in force states it, and the
    /// three interrupt enables with DMAEN and LAST as `enables` names
    /// them. What an engine switches inside a tenure is several of these
    /// bits at once, and one read-modify-write each would be two bus
    /// accesses a bit where this is one store.
    [[gnu::always_inline]] static void control2(uint8_t freq_mhz, uint16_t enables) {
        regs().CTLR2 = static_cast<uint16_t>(
            (freq_mhz & i2c_freq_mask) |
            (enables & (i2c_iterren | i2c_itevten | i2c_itbufen | i2c_dmaen | i2c_last)));
    }

private:
    [[gnu::always_inline]] static void ctlr1(uint16_t bit, bool on) {
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

/// The column's refusals at run time, shared by the two init()s: a
/// column on the debug port's pads while the probe owns them, or on the
/// USB pads while the USB transceiver does (the file header). True when
/// the column may be taken.
template <I2cPins pins>
inline bool i2c_column_free() {
    if constexpr (i2c_pins_on_debug_port(pins)) {
        if (Afio::debug_port_enabled()) {
            return false;
        }
    }
    if constexpr (i2c_pins_on_usb_pads(pins)) {
        if (Afio::usb_pads_taken()) {
            return false;
        }
    }
    return true;
}

// =============================================================================
// The host engine
// =============================================================================

/// A DMA block the engines could not finish: the engine's own code, in
/// the range util/bus_master.hpp leaves to engines and above the four
/// i2c_bus.hpp holds.
inline constexpr uint8_t i2c_dma_fault = bus_engine_status + 4;

/**
 * I2cHost<n, pins, TxEngine, RxEngine, Placement>
 *
 * The engine I2cBus (util/i2c_bus.hpp = BusMaster) drives. One Request
 * is one bus tenure: START, the address, tx_len bytes written, then - if
 * rx_len is not zero - a repeated START, the address again with the read
 * bit and rx_len bytes taken with the last one NACKed, then STOP. Both
 * spans empty is the probe: START, address, STOP - the ACK on the
 * address is the answer. Every outcome the wire can give comes back as
 * i2c_bus.hpp's codes through TransferDone{status()}.
 *
 *   using Bus = brio::I2cHost<1>;   // the part's first column
 *   using I2c = brio::I2cBus<Bus, P, 4, brio::BusPassThrough, brio::ticks_from_ms<P>(20)>;
 *   Bus::init(clock);               // after the clock task; false: no block, no column
 *
 *   Bus::Request r{};
 *   r.addr = 0x48;
 *   r.tx = brio::lend<brio::Lease::reply>(reg); r.tx_len = 1;
 *   r.rx = brio::lend<brio::Lease::reply>(value); r.rx_len = 2;   // write-then-read
 *   r.reply = brio::reply_to<Sensor, brio::I2cDone>();
 *   brio::post<I2c>(r);
 *
 *   extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() {
 *       if (Bus::isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
 *   }
 *   extern "C" BRIO_CH32_INTERRUPT void i2c1_er_handler() {
 *       if (Bus::error_isr()) { brio::post<I2c>(brio::TransferDone{Bus::status()}); }
 *   }
 *
 * `speed` names a row of the timing table init() solves for the bus
 * clock; a speed the clock cannot produce is answered i2c_rejected inside
 * start(), no byte moved - the one synchronous completion of an I2C
 * engine, and the arbiter replies with status() for it.
 *
 * The engine slots: `DmaTxEngine<6>` and `DmaRxEngine<7>`, both or
 * neither (the file header says what each carries).
 */
template <uint8_t n, I2cPins pins = i2c1_default_pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine, typename Placement = InFlash>
class I2cHost {
    using S = I2c<n>;

    static_assert(HotPlacement<Placement, Ch32x035Platform<>>,
                  "brio I2cHost: the placement is util/placement.hpp's InFlash (the default) or InRam");

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from ch32x035/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio I2cHost: name both DMA engines or neither - a write-then-read "
                  "tenure needs both directions inside one tenure");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "brio I2cHost: the two engines must ride two different DMA channels");
    static_assert(dma_engine_channel<TxEngine>() == 0u ||
                      (dma_engine_channel<TxEngine>() == S::dma_tx_channel &&
                       dma_engine_channel<RxEngine>() == S::dma_rx_channel),
                  "brio I2cHost: on this series the channel IS the request (RM figure 9-1) - "
                  "I2C1 transmits on DMA channel 6 and receives on channel 7");
    static_assert(i2c_pins_valid(pins),
                  "brio I2cHost: these I2C pads are not a bus on this part - SCL and SDA must "
                  "be one column of I2C1_RM (afio.hpp), both bonded by this package and "
                  "neither sharing its pin with another pad (parts/<part>.hpp)");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;

public:
    I2cHost() = delete;

    using Resource = S;
    static constexpr I2cPins pin_pads = pins;
    static constexpr bool has_engines = TxEngine::present;

    struct Request {
        uint8_t addr;          ///< 7-bit client address (unshifted)
        /// Bytes written after START, lent until the reply lands (may be
        /// null if tx_len == 0).
        Borrowed<const uint8_t, Lease::reply> tx;
        uint8_t tx_len;
        /// Where the bytes read after the (repeated) START+R land; lent
        /// until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint8_t rx_len;
        ReplyTo<I2cDone> reply;
        I2cSpeed speed = I2cSpeed::standard_100k;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------

    /**
     * Bring the instance up as a bus host: a setup verb, main(), after the
     * clock task and before interrupts are enabled globally. `clock` is
     * the app's Clock tag, the one truth of the bus clock (HCLK: every bus
     * runs at it on this series; the timing is computed from the rate the
     * type claims, nothing is read from the hardware). `duty` is fast
     * mode's shape. Both speeds are solved here and at rebase(); one the
     * clock cannot produce is marked unreachable (speed_ok()) and a
     * Request naming it is answered i2c_rejected. The pads are handed to
     * the peripheral here and the two PFIC lines enabled; the vectors are
     * the program's to bind.
     *
     * False, and the pads untouched, when not even standard_100k is legal
     * - every HCLK under 4 MHz (15.9.2) -, when the column is refused at
     * run time (the debug port's pads with the probe on them, the USB pads
     * under USB_IOEN), or when the block does not hold what was written
     * into it (the lot note, the file header).
     */
    template <typename Clock>
    static bool init(Clock clock, I2cDuty duty = I2cDuty::ratio_2) {
        static_assert(clock_follows<Clock, I2cHost>(),
                      "this I2cHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its timing table would go stale on a "
                      "clock change");
        if (!i2c_column_free<pins>()) {
            return false;
        }
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        duty_ = duty;
        S::bus_clock(true);
        S::reset();
        rebase(clock_hz(clock));
        if (!valid_[0]) {
            S::bus_clock(false);
            return false;
        }
        applied_ = I2cSpeed::standard_100k;
        S::timing(table_[0]);
        if (S::regs().CKCFGR != table_[0].ckcfgr) {
            S::bus_clock(false);   // no block behind the address: the lot note
            return false;
        }
        (void)S::remap(pins.remap);
        S::enable();
        S::ack(false);
        if constexpr (has_engines) {
            // Both at high (DmaPriority's rule): the host does not overrun.
            // A byte its receive channel has not taken leaves RxNE up and
            // the next one in the shifter, BTF, and SCL held low until DATAR
            // is read; a byte its transmit channel has not written holds SCL
            // the same way - a starved channel stretches the bus and loses
            // nothing (15.6). The transmit engine arms for its ERRORS alone:
            // BTF on the event line ends a plain write (the file header).
            TxEngine::arm(S::data_address(), TxEngine::flag_error, DmaPriority::high);
            RxEngine::arm(S::data_address(), DmaPriority::high);
        }
        // The pads go to the peripheral only now, open drain by themselves
        // (8.2.8): the wire's pull-ups own the idle level.
        SclPin::function();
        SdaPin::function();
        t_.status = i2c_ok;
        t_.phase = Phase::idle;
        // The error vector on, the event vector OFF until a tenure starts.
        S::control2(t_.freq, idle_enables);
        Pfic::enable(S::event_irq());
        Pfic::enable(S::error_irq());
        return true;
    }

    /// The bus clock changed (DynamicClock fan-out): both rows solved
    /// again and the applied one rewritten. The bus must be idle.
    static void rebase(uint32_t hz) {
        pclk_hz_ = hz;
        for (uint8_t i = 0; i < i2c_speed_count; ++i) {
            const auto t = i2c_timing_for(hz, static_cast<I2cSpeed>(i), duty_);
            valid_[i] = t.has_value();
            table_[i] = t.value_or(I2cTiming{});
        }
        t_.freq = table_[0].freq_mhz;   // FREQ is HCLK's, the same on both rows
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

    /// Begin a tenure (I2cBus calls it from main context). False whenever
    /// the wire moves: the tenure completes on the event vector and a
    /// TransferDone{status()} follows. True only for the one refusal that
    /// moves nothing - a speed the clock cannot produce, answered
    /// i2c_rejected through status() - the I2cHost contract
    /// (docs/design/i2c-bus.md).
    static bool start(const Request& r) {
        if (!speed_ok(r.speed)) {
            t_.status = i2c_rejected;
            t_.phase = Phase::idle;
            return true;
        }
        // The fields a tenure walks, and nothing else of the Request: the
        // reply is the arbiter's, and a pointer and a count are what the
        // handlers advance.
        t_.addr = r.addr;
        t_.tx = r.tx.get();
        t_.tx_left = r.tx_len;
        t_.rx = r.rx.get();
        t_.rx_len = r.rx_len;
        t_.rx_left = r.rx_len;
        t_.status = i2c_ok;
        t_.phase = (r.tx_len == 0u && r.rx_len != 0u) ? Phase::start_rx : Phase::start_tx;
        // The last tenure's STOP may still be on its way out: STOP stands in
        // CTLR1 until the condition is on the wire ("cleared by hardware
        // when a Stop condition is detected", 15.9.1) - about a bit after
        // the completion edge (measured) - and a client that stretches the
        // clock after the last acknowledge holds it there.
        // No CTLR1 write may land while it stands - the F1 lineage's rule,
        // kept though a read-modify-write under it left no trace on this die
        // (measured, docs/ch32x035/i2c.md): the wait costs a bit - and no
        // flag marks a host's own STOP leaving (STOPF is the client half's),
        // so it is waited for here, before anything writes CTLR1, a speed
        // change's PE cycle included, bounded in time (stop_drain_us). A clock held past the
        // bound is a wedge: the tenure PARKS - no START and no vector - for
        // the per-bus timeout to answer.
        if (!wait_for_us([] { return !S::stopping(); }, stop_drain_us)) {
            return false;
        }
        // STAR1 is read before STAR2 and the CTLR1 store below: a STOPF or
        // an ADDR the client half raised while the event vector was off is
        // cleared by those accesses - the chapter's own sequences - instead
        // of entering the vector under the START. BUSY clears a moment
        // after our STOP is seen; a bus still busy after bus_free_us is
        // someone else's - the START set into it goes out at the block's
        // own pace or stays pending (the file header), and either way the
        // tenure is answered - or the peripheral's own stuck bit
        // (unwedge_busy).
        (void)S::status1();
        if (!wait_for_us([] { return !S::busy(); }, bus_free_us)) {
            (void)unwedge_busy();
        }
        apply(r.speed);
        // The event vector on, the buffer vector and the DMA requests off:
        // one store. Then ACK and POS down and START up: one read-modify-
        // write.
        S::control2(t_.freq, idle_enables | i2c_itevten);
        I2cRegs& reg = S::regs();
        reg.CTLR1 = static_cast<uint16_t>((reg.CTLR1 & ~(i2c_ack | i2c_pos)) | i2c_start);
        return false;
    }

    /**
     * 15.9.1's own case: SWRST "resets the I2C module when no stop
     * condition is detected on the bus but the busy bit is 1". When the
     * bounded wait for BUSY runs out and the wire says idle - both lines
     * high - the bit is the peripheral's own and the chapter's reset is
     * taken; a bus another master really holds never has both lines high
     * for long, and is never reset out from under it. SWRST takes CTLR2
     * with it, so the whole configuration is written again. True when the
     * machine was reset.
     */
    static bool unwedge_busy() {
        if (!S::busy() || !SclPin::read() || !SdaPin::read()) {
            return false;
        }
        S::software_reset();
        S::timing(table_[static_cast<uint8_t>(applied_)]);
        S::enable();
        S::ack(false);
        S::control2(t_.freq, idle_enables);
        ++unwedged_;
        return true;
    }

    /// How many times the bit above has been reset out of the way - a
    /// count, not a promise, and what a suite prints to say whether the
    /// case is real on this silicon.
    static uint16_t unwedges() { return unwedged_; }

    /// The engine's completion status, for the TransferDone payload: read
    /// in the vector right after isr() or error_isr() answered true, or
    /// after a synchronous start().
    static uint8_t status() { return t_.status; }

    /// Is a tenure in flight? The engine's phase, not the wire's - what a
    /// sleep site asks.
    static bool busy() { return t_.phase != Phase::idle; }

    /// The event vector's body - call from i2c1_ev's handler, from nowhere else. Returns true
    /// when the tenure just completed: the edge the app's glue posts
    /// TransferDone on. The vector calls the placement's trampoline (the
    /// file header).
    [[gnu::always_inline]] static bool isr() {
        return Hot<Placement>::run([] { return event_entry(); });
    }

private:
    /// The event entry's path, written once: isr() runs it through the
    /// host's placement.
    [[gnu::always_inline]] static bool event_entry() {
        const uint16_t s1 = S::status1();
        // STOPF is the client half's flag - a STOP seen after a START this
        // host did not issue: another master's, or the one unstick() makes
        // by hand. Its sequence (EVT4) ends in a CTLR1 write, which must not
        // land while START or STOP stands, so it waits for them to leave.
        if ((s1 & i2c_stopf) != 0u && !S::starting() && !S::stopping()) {
            S::clear_stopf();
        }
        switch (t_.phase) {
            case Phase::idle:
                // Nothing in flight: an ADDR here is the client half's too,
                // and its sequence (STAR1 read above, then STAR2) writes
                // nothing.
                if ((s1 & i2c_addr) != 0u) {
                    (void)S::status2();
                }
                return false;

            case Phase::start_tx:
                // EVT5: STAR1 read above, then the address into DATAR.
                if ((s1 & i2c_sb) == 0u) {
                    return false;
                }
                S::data(static_cast<uint8_t>(t_.addr << 1));
                t_.phase = Phase::addr_tx;
                return false;

            case Phase::start_rx:
            case Phase::restart:
            case Phase::refused:
                if ((s1 & i2c_sb) == 0u) {
                    return false;
                }
                if (t_.phase == Phase::refused ||
                    (t_.phase == Phase::restart && (s1 & i2c_af) != 0u)) {
                    return close_refused();
                }
                prime_receive();
                S::data(static_cast<uint8_t>((t_.addr << 1) | 1u));
                t_.phase = Phase::addr_rx;
                return false;

            case Phase::addr_tx:
                if ((s1 & i2c_addr) == 0u) {
                    return false;
                }
                if (t_.tx_left == 0u) {
                    // The probe: acknowledged, and that was the question.
                    // STAR1 was read above, so STAR2 alone clears ADDR.
                    (void)S::status2();
                    S::stop();
                    return finish(i2c_ok);
                }
                if constexpr (has_engines) {
                    // A plain write of three bytes or more on the transmit
                    // engine; the write half of a write-then-read stays on
                    // the pump (the file header).
                    if (t_.rx_len == 0u && t_.tx_left >= dma_min_write) {
                        S::control2(t_.freq, idle_enables | i2c_itevten | i2c_dmaen);
                        (void)TxEngine::start(std::span<const uint8_t>(t_.tx, t_.tx_left));
                        (void)S::status2();
                        t_.phase = Phase::tx_dma;
                        return false;
                    }
                }
                (void)S::status2();   // ADDR cleared: EVT8_1 follows
                return first_bytes();

            case Phase::tx:
                return transmit_step(s1);

            case Phase::tx_dma:
                return transmit_dma_step(s1);

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
                // The void write's address acknowledged: ADDR cleared (STAR1
                // read above, then STAR2), and the STOP requested behind it
                // goes out.
                if ((s1 & i2c_addr) == 0u) {
                    return false;
                }
                (void)S::status2();
                return finish(i2c_nack_data);
        }
        return false;
    }

public:
    /// The error vector's body - call from i2c1_er's handler, from nowhere else. Every error
    /// ends the tenure with its i2c_bus.hpp code; a NACK is on the address
    /// or on a byte by the phase it landed in. True when the tenure just
    /// completed.
    [[gnu::always_inline]] static bool error_isr() {
        const uint16_t s1 = S::status1();
        const uint16_t errs = s1 & i2c_errors;
        if (errs == 0u) {
            return false;
        }
        S::clear_errors(errs);
        if (t_.phase == Phase::idle) {
            return false;
        }
        uint8_t st = i2c_bus_error;
        if ((errs & i2c_af) != 0u && t_.phase == Phase::restart) {
            // The last written byte refused, its repeated START already
            // requested (request_restart()): the NACK is the byte's, and the
            // Sr the controller generates anyway is waited for - its SB
            // closes the tenure (close_refused(), the file header). Nothing
            // is written here, so nothing races the controller.
            t_.phase = Phase::refused;
            return false;
        }
        if ((errs & i2c_af) != 0u && t_.phase == Phase::closing) {
            // The void write's address refused too: its STOP is requested.
            return finish(i2c_nack_data);
        }
        if ((errs & i2c_af) != 0u) {
            const bool on_address = t_.phase == Phase::addr_tx || t_.phase == Phase::addr_rx ||
                                    t_.phase == Phase::start_tx || t_.phase == Phase::start_rx;
            st = on_address ? i2c_nack_addr : i2c_nack_data;
            S::stop();   // 15.5.2: in master mode the software generates the STOP
        } else if ((errs & i2c_arlo) != 0u) {
            st = i2c_arb_lost;   // 15.5.3: the hardware released the bus already
        } else {
            S::stop();   // BERR in master mode: 15.5.1 leaves the abort to the user code
        }
        if constexpr (has_engines) {
            put_engines_away();
        }
        return finish(st);
    }

    /// The DMA channels' interrupt body - call from BOTH channels'
    /// vectors. The receive block completing ends a read phase (STOP after
    /// it); the transmit channel interrupts on an error alone (BTF on the
    /// event vector ends a write). A transfer error ends the tenure with
    /// i2c_dma_fault. INTFR is read once for both. True when the tenure
    /// just completed.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint32_t intfr = RxEngine::block_flags();
            const uint8_t tx = TxEngine::service(intfr);
            const uint8_t rx = RxEngine::service(intfr);
            if (((tx & TxEngine::flag_error) | (rx & RxEngine::flag_error)) != 0u) {
                put_engines_away();
                S::stop();
                return finish(i2c_dma_fault);
            }
            if ((rx & RxEngine::flag_complete) != 0u && t_.phase == Phase::rx_dma) {
                // The block's last byte NACKed under LAST; the STOP now, and
                // the DMA requests down with the vectors in finish()'s store.
                S::stop();
                return finish(i2c_ok);
            }
        }
        return false;
    }

    /**
     * The classic bus unstick: up to nine SCL pulses and a STOP, by hand,
     * with the pads reclaimed from the peripheral for the duration.
     * recover() fixes the peripheral, this fixes the wire. This series has
     * no open-drain GPIO (8.3.1.1), so a line is driven low as a push-pull
     * output over a clear output bit and released as a floating input - a
     * line is never driven high, so a client holding it low is never
     * fought. Returns the pulses it took a stuck client to release SDA
     * (0 = the wire was never stuck), or 0xFF when nine pulses and a STOP
     * left SDA low - a short, not a client. A healthy wire is left alone.
     */
    static uint8_t unstick() {
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        (void)SclPin::input();
        (void)SdaPin::input();

        uint8_t released_at = 0xFF;
        bool free_now = SdaPin::read();
        if (!free_now) {
            uint8_t pulses = 0;
            for (uint8_t i = 0; i < 9u && released_at == 0xFF; ++i) {
                SclPin::output(false);
                spin_half_bit();
                (void)SclPin::input();
                spin_half_bit();
                ++pulses;
                if (SdaPin::read()) {
                    released_at = pulses;
                }
            }
            // A START and a STOP with SCL standing high: SDA pulled low
            // (every client resets at the START) and released (the STOP).
            SdaPin::output(false);
            spin_half_bit();
            (void)SdaPin::input();
            spin_half_bit();
            free_now = SdaPin::read();
        } else {
            released_at = 0;
        }

        SclPin::function();
        SdaPin::function();
        S::clear_errors(i2c_errors);
        // The START and STOP made by hand are seen by the instance's own
        // client half, which may raise STOPF with nothing in flight; its
        // sequence clears it here (EVT4), START and STOP down.
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

    /// Put the engine back where start() is legal - the verb a timed I2cBus
    /// calls on a tenure that never answered: the engines put away, SWRST
    /// pulsed (the chapter's own verb for a controller stuck BUSY), the
    /// timing rewritten, PE back. The wire stays the application's
    /// (unstick()).
    static bool recover() {
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        if constexpr (has_engines) {
            put_engines_away();
        }
        t_.phase = Phase::idle;
        S::software_reset();
        S::timing(table_[static_cast<uint8_t>(applied_)]);
        S::enable();
        S::ack(false);
        S::control2(t_.freq, idle_enables);   // the event vector waits for start()
        Pfic::enable(S::event_irq());
        Pfic::enable(S::error_irq());
        return true;
    }

    /**
     * The block given back. The reset line before the gate
     * (docs/design/dma.md, the release contract): on this die a raised DMA
     * request can outlive its peripheral's DMA enable, its enable and a
     * closed gate unless the block is reset (measured on USART3 and on the
     * timers, docs/ch32x035/dma.md, "A released requester"), and channels 6
     * and 7 are ORs this block shares with USART2 and the timers. This
     * block's own request, raised under DMAEN by a read stalled over a dead
     * channel, measured not to outlive its gate (docs/ch32x035/i2c.md): the
     * pulse is the contract's - the vendor's own DeInit - two stores that
     * leave nothing whatever the block holds.
     */
    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        S::disable();
        S::reset();
        S::bus_clock(false);
        t_.phase = Phase::idle;
        SclPin::release();
        SdaPin::release();
    }

private:
    /// `restart`: the repeated START requested behind the last written
    /// byte, SB not yet seen - the phase in which a NACK is that byte's.
    /// `refused`: that NACK taken, the Sr still coming; `closing`: the void
    /// write that ends a refused tenure, its address out and its STOP
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
    }(), "brio I2cHost: the DMA engines must carry uint8_t elements - DATAR's data is a "
         "byte (15.9.5)");

    /// What CTLR2 holds between tenures besides FREQ: the error vector
    /// alone. A tenure adds the event vector, and inside it the buffer
    /// vector or the DMA requests (control2(), one store each time).
    static constexpr uint16_t idle_enables = i2c_iterren;

    /// The shortest plain write the transmit engine carries: EVT8_1's
    /// entry takes two bytes with no interrupt (first_bytes()), where an
    /// engine adds its block start.
    static constexpr uint8_t dma_min_write = 3;

    /// How many STAR1 reads the entry that cleared ADDR spends waiting for
    /// TxE before it leaves the byte to the buffer vector. Enough on this
    /// die: a one- and a two-byte write take three entries and a register
    /// index before a read none of its own, both bytes loaded in the ADDR
    /// entry (measured, docs/ch32x035/i2c.md).
    static constexpr uint8_t txe_polls = 16;

    /// The tenure is over: the status, the phase, and CTLR2 back to the
    /// error vector alone - the event and buffer vectors and the DMA
    /// requests down in one store, so the BTF a STOP leaves standing does
    /// not re-enter the vector.
    [[gnu::always_inline]] static bool finish(uint8_t st) {
        t_.status = st;
        t_.phase = Phase::idle;
        S::control2(t_.freq, idle_enables);
        return true;
    }

    /// TxE within a bound of STAR1 reads.
    [[gnu::always_inline]] static bool txe_soon() {
        for (uint8_t i = 0; i < txe_polls; ++i) {
            if (S::flag(i2c_txe)) {
                return true;
            }
        }
        return false;
    }

    /// The repeated START of a write-then-read, requested while the last
    /// written byte is still in the shifter: the peripheral generates it at
    /// the end of that byte (15.9.1's START bit). The buffer vector goes
    /// down, the TxE that brought this here having no more to say.
    [[gnu::always_inline]] static void request_restart() {
        S::control2(t_.freq, idle_enables | i2c_itevten);
        t_.phase = Phase::restart;
        S::start();
    }

    /// EVT5 of the Sr behind a refused last byte: the address with the
    /// WRITE bit, which clears SB, and STOP requested behind it - "after
    /// the current byte transfer" (15.9.1) - so the wire carries a void
    /// write and the controller is left as a plain write leaves it. AF is
    /// cleared here too, for the case in which this vector saw it first.
    [[gnu::always_inline]] static bool close_refused() {
        S::clear_errors(i2c_af);
        S::data(static_cast<uint8_t>(t_.addr << 1));
        S::stop();
        t_.phase = Phase::closing;
        return false;
    }

    /// EVT8_1 and EVT8 in the entry that cleared ADDR (the file header):
    /// the data register and the shifter take the first two bytes, and a
    /// one-byte index before a read has its repeated START requested here.
    [[gnu::always_inline]] static bool first_bytes() {
        t_.phase = Phase::tx;
        if (!txe_soon()) {
            // Not seen in the bound: the buffer vector takes the byte.
            S::control2(t_.freq, idle_enables | i2c_itevten | i2c_itbufen);
            return false;
        }
        S::data(*t_.tx++);
        --t_.tx_left;
        if (t_.tx_left != 0u) {
            if (txe_soon()) {
                S::data(*t_.tx++);
                --t_.tx_left;
            }
        } else if (t_.rx_len != 0u && txe_soon()) {
            request_restart();
            return false;
        }
        if (t_.tx_left != 0u || t_.rx_len != 0u) {
            // TxE: the next byte, or the repeated START behind the last.
            S::control2(t_.freq, idle_enables | i2c_itevten | i2c_itbufen);
        }
        // Otherwise every byte is in and BTF (the event vector) ends it.
        return false;
    }

    /// EVT8 on the buffer vector, and EVT8_2 on the event vector. A read
    /// half to follow has its repeated START requested on the TxE that says
    /// the last byte went into the shifter, and a TxE served after its
    /// byte's end, with BTF up, takes the same act.
    [[gnu::always_inline]] static bool transmit_step(uint16_t s1) {
        if ((s1 & i2c_txe) == 0u) {
            return false;   // BTF implies TxE
        }
        if (t_.tx_left != 0u) {
            S::data(*t_.tx++);
            --t_.tx_left;
            if (t_.tx_left == 0u && t_.rx_len == 0u) {
                // BTF carries the end.
                S::control2(t_.freq, idle_enables | i2c_itevten);
            }
            return false;
        }
        if (t_.rx_len != 0u) {
            request_restart();
            return false;
        }
        if ((s1 & i2c_btf) != 0u) {
            S::stop();   // EVT8_2
            return finish(i2c_ok);
        }
        return false;
    }

    /**
     * The engine's plain write on the event vector: BTF, the last byte's
     * end. BTF proves the block's end only with the channel's count at
     * zero: under a live channel the count is long at zero by then - the
     * channel refills DATAR within a few bus cycles of each TxE, a byte
     * time ahead of BTF - and a count still standing is a channel that
     * stopped serving (a request another peripheral left held on the
     * channel's OR, docs/design/dma.md): the tenure ends i2c_dma_fault, its
     * STOP requested. One CNTR load on the one BTF entry of a write.
     */
    [[gnu::always_inline]] static bool transmit_dma_step(uint16_t s1) {
        if constexpr (has_engines) {
            if ((s1 & i2c_btf) == 0u) {
                return false;
            }
            if (TxEngine::progress().remaining != 0u) {
                put_engines_away();
                S::stop();
                return finish(i2c_dma_fault);
            }
            (void)TxEngine::complete();
            S::stop();   // EVT8_2; DMAEN goes down in finish()'s store
            return finish(i2c_ok);
        } else {
            (void)s1;
            return false;
        }
    }

    /// The ACK/POS arrangement the read phase's length wants, set before
    /// the address goes out (15.3's receive procedures), in one CTLR1
    /// read-modify-write (START is down: SB is what brought us here).
    [[gnu::always_inline]] static void prime_receive() {
        uint16_t bits = t_.rx_len != 1u ? i2c_ack : uint16_t{0};
        if (t_.rx_len == 2u && !dma_serves_rx()) {
            bits = static_cast<uint16_t>(bits | i2c_pos);
        }
        I2cRegs& reg = S::regs();
        reg.CTLR1 = static_cast<uint16_t>((reg.CTLR1 & ~(i2c_ack | i2c_pos)) | bits);
    }

    /// EVT6 on a read: the procedure by count. STAR1 was read by the entry,
    /// so STAR2 alone clears ADDR.
    [[gnu::always_inline]] static bool begin_receive() {
        if constexpr (has_engines) {
            if (dma_serves_rx()) {
                // LAST: the block's last byte is NACKed. The event vector goes
                // down for the block: dma_isr() ends the phase and the event
                // vector has nothing to serve in it - and over a channel that
                // does not serve its request RxNE and BTF would stand and
                // re-enter it for ever, the thread and the bus's timeout
                // starved. The error vector stays.
                S::control2(t_.freq, idle_enables | i2c_dmaen | i2c_last);
                (void)RxEngine::start(std::span<uint8_t>(t_.rx, t_.rx_len));
                (void)S::status2();
                t_.phase = Phase::rx_dma;
                return false;
            }
        }
        t_.phase = Phase::rx;
        if (t_.rx_len == 1u) {
            // EVT6_1: ACK is already clear (prime_receive): clear ADDR, then
            // STOP at once; the byte lands on RxNE.
            (void)S::status2();
            S::stop();
            S::control2(t_.freq, idle_enables | i2c_itevten | i2c_itbufen);
            return false;
        }
        if (t_.rx_len == 2u) {
            // POS and ACK were set: clear ADDR, then ACK off - the NACK lands
            // on the second byte; one BTF delivers both. POS stays up until
            // the next start() takes it down with ACK.
            (void)S::status2();
            S::ack(false);
            return false;
        }
        (void)S::status2();
        if (t_.rx_len > 3u) {
            // RxNE while more than three remain.
            S::control2(t_.freq, idle_enables | i2c_itevten | i2c_itbufen);
        }
        return false;
    }

    /// The receive pump, per the count's procedure.
    [[gnu::always_inline]] static bool receive_step(uint16_t s1) {
        if (t_.rx_len == 1u) {
            if ((s1 & i2c_rxne) == 0u) {
                return false;
            }
            *t_.rx = S::data();
            return finish(i2c_ok);
        }
        if (t_.rx_len == 2u) {
            if ((s1 & i2c_btf) == 0u) {
                return false;
            }
            S::stop();
            t_.rx[0] = S::data();
            t_.rx[1] = S::data();
            return finish(i2c_ok);
        }
        if (t_.rx_left > 3u) {
            if ((s1 & i2c_rxne) != 0u) {
                *t_.rx++ = S::data();
                --t_.rx_left;
                if (t_.rx_left == 3u) {
                    // BTF from here on.
                    S::control2(t_.freq, idle_enables | i2c_itevten);
                }
            }
            return false;
        }
        if ((s1 & i2c_btf) == 0u) {
            return false;
        }
        if (t_.rx_left == 3u) {
            // N-2 in DATAR, N-1 in the shifter: NACK the one to come, read
            // N-2, which lets N in.
            S::ack(false);
            *t_.rx++ = S::data();
            --t_.rx_left;
            return false;
        }
        // Two left: N-1 in DATAR, N in the shifter.
        S::stop();
        *t_.rx++ = S::data();
        (void)wait_until([] { return S::flag(i2c_rxne); });
        *t_.rx = S::data();
        return finish(i2c_ok);
    }

    static bool dma_serves_rx() {
        if constexpr (has_engines) {
            return t_.rx_len >= 2u;
        } else {
            return false;
        }
    }
    static void put_engines_away() {
        if constexpr (has_engines) {
            S::dma(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();   // the binding stands: the next start() needs no arm()
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

    /// How long start() waits for the last tenure's STOP to leave the wire.
    /// A STOP takes a bit period; what can hold it longer is a slow client
    /// stretching the clock after the last acknowledge while it digests a
    /// write, and five milliseconds - a fifth of SMBus's 25 ms clock-low
    /// limit - is the bound. What start() spends here is the kernel
    /// dispatch's time, which is why it is bounded at all.
    static constexpr uint16_t stop_drain_us = 5'000;

    /// How long start() then waits for BUSY to clear: our STOP seen by the
    /// bus takes a moment, another device's traffic takes longer.
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
    [[gnu::always_inline]] static bool wait_until(Pred pred) {
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }

    /// Half a standard-mode bit, for the unstick.
    static void spin_half_bit() { (void)delay_us(delay_rate_, 5); }

    /// The tenure in flight as the handlers walk it, and CTLR2's FREQ, in
    /// ONE object: a handler forms one base address and reaches every
    /// field by an offset from it.
    struct Tenure {
        const uint8_t* tx;    ///< the next byte to write
        uint8_t* rx;          ///< where the next byte read lands
        uint8_t tx_left;
        uint8_t rx_len;       ///< the read phase's length: its procedure
        uint8_t rx_left;
        uint8_t addr;
        uint8_t freq;         ///< CTLR2.FREQ in force
        uint8_t status;
        volatile Phase phase;
    };
    static inline Tenure t_{nullptr, nullptr, 0, 0, 0, 0, 0, i2c_ok, Phase::idle};
    // The configuration.
    static inline I2cSpeed applied_ = I2cSpeed::standard_100k;
    static inline I2cDuty duty_ = I2cDuty::ratio_2;
    static inline uint16_t unwedged_ = 0;
    static inline uint32_t pclk_hz_ = 0;
    static inline I2cTiming table_[i2c_speed_count]{};
    static inline bool valid_[i2c_speed_count]{};
    static inline DelayRate delay_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/// What the client's ISR body reports: one thing per call, the app's glue
/// acts on it.
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
/// three enables down for a polled client, which is what an instrument
/// that must spend a commanded hold inside a tenure needs: the flags then
/// belong to the loop alone.
struct I2cClientOptions {
    bool no_stretch = false;
    bool interrupts = true;
};

/**
 * I2cClient<n, pins>
 *
 * The target side. The peripheral matches the own address(es), ACKs it,
 * raises ADDR and stretches SCL until the sequence that clears it runs
 * (15.4, EVT1); every byte then comes on RxNE or is asked for on TxE, with
 * the clock stretched while the flag stands unless NOSTRETCH (15.6). A
 * read tenure ends with the host's NACK (AF, on the error vector, EVT3_2);
 * a write tenure with STOPF (EVT4).
 *
 *   extern "C" BRIO_CH32_INTERRUPT void i2c1_ev_handler() { act(Peer::service()); }
 *   extern "C" BRIO_CH32_INTERRUPT void i2c1_er_handler() { act(Peer::error_service()); }
 */
template <uint8_t n, I2cPins pins = i2c1_default_pins>
class I2cClient {
    using S = I2c<n>;

    static_assert(i2c_pins_valid(pins),
                  "brio I2cClient: these I2C pads are not a bus on this part - SCL and SDA must "
                  "be one column of I2C1_RM (afio.hpp), both bonded by this package and "
                  "neither sharing its pin with another pad (parts/<part>.hpp)");

    using SclPin = Pin<pins.scl.port, pins.scl.pin>;
    using SdaPin = Pin<pins.sda.port, pins.sda.pin>;

public:
    I2cClient() = delete;

    using Resource = S;
    static constexpr I2cPins pin_pads = pins;

    /// Bring the instance up as a bus target. FREQ must still state the
    /// bus clock (the block's own timing comes from it even as a target),
    /// so 15.9.2's window applies here too. False under the host's three
    /// run-time refusals (the clock, the column, the block not there).
    template <typename Clock>
    static bool init(Clock clock, const I2cAddressConfig& addr,
                     const I2cClientOptions& opts = {}) {
        if (!i2c_column_free<pins>()) {
            return false;
        }
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        S::bus_clock(true);
        S::reset();
        const auto t = i2c_timing_for(clock_hz(clock), I2cSpeed::standard_100k);
        if (!t) {
            S::bus_clock(false);
            return false;
        }
        S::timing(*t);
        if (S::regs().CKCFGR != t->ckcfgr || !S::addresses(addr)) {
            S::bus_clock(false);
            return false;
        }
        (void)S::remap(pins.remap);
        S::no_stretch(opts.no_stretch);
        S::enable();
        S::ack(true);   // answer the own address
        SclPin::function();
        SdaPin::function();
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

    /// An address match is pending (SCL held meanwhile, unless NOSTRETCH).
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
    /// (15.5.2: the hardware released the bus).
    static bool host_nacked() { return S::flag(i2c_af); }
    static void clear_nack() { S::clear_errors(i2c_af); }
    static bool overrun() { return S::flag(i2c_ovr); }
    static void clear_overrun() { S::clear_errors(i2c_ovr); }
    static uint16_t flags() { return S::status1(); }

    /**
     * Throw away a byte the host never took. A target transmitter is
     * asked for one byte more than the controller takes: DATAR empties
     * into the shifter and TxE rises again, so the byte standing in DATAR
     * when the closing NACK arrives is never clocked out and would be the
     * first byte of the next read tenure. The chapter has no verb that
     * discards it; a PE cycle does - PE clear resets TxE, RxNE, BTF and
     * the flags (15.9.6's "or by hardware when PE is 0") while the
     * addresses and the timing stay - and ACK, which PE clear clears too
     * (15.9.1), goes back up here. True when there was a byte to drop;
     * false when DATAR was already empty (TxE standing). Asked at the
     * moment after the host's closing NACK, where the question is real.
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

    /// The event vector's body: one event per call, the flag consumed where
    /// the chapter's sequence consumes it (ADDR and STOPF here; RxNE by
    /// take(), TxE by give(), which the glue owes).
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

    /// The error vector's body: the host's NACK (the normal end of a read)
    /// or a real error, the flag cleared.
    [[gnu::always_inline]] static I2cClientEvent error_service() {
        const uint16_t s1 = S::status1();
        const uint16_t errs = s1 & i2c_errors;
        if (errs == 0u) {
            return I2cClientEvent::none;
        }
        S::clear_errors(errs);
        return (errs & i2c_af) != 0u ? I2cClientEvent::nacked : I2cClientEvent::error;
    }

    /// The direction of the tenure the last `addressed` opened: true = the
    /// host reads, this side transmits.
    static bool host_reads() { return host_reads_; }

    /// The block given back, the reset line before the gate as the host's.
    static void release() {
        Pfic::disable(S::event_irq());
        Pfic::disable(S::error_irq());
        S::disable();
        S::reset();
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
static_assert(i2c_scl_hz(48'000'000UL, *i2c_timing_for(48'000'000UL, I2cSpeed::standard_100k)) ==
              100'000UL);
static_assert(i2c_scl_hz(48'000'000UL, *i2c_timing_for(48'000'000UL, I2cSpeed::fast_400k)) ==
              400'000UL);
// The reset HCLK, 8 MHz: CCR 40 and 7 (381 kHz, rounded up from 6.67).
static_assert(i2c_timing_for(8'000'000UL, I2cSpeed::standard_100k)->ckcfgr == 40u);
static_assert(i2c_timing_for(8'000'000UL, I2cSpeed::fast_400k)->ckcfgr == (i2c_fs | 7u));
// The window (15.9.2): 4 MHz is legal, 3 MHz - the HPRE ladder's next rung
// down - is not.
static_assert(i2c_timing_for(4'000'000UL, I2cSpeed::standard_100k)->ckcfgr == 20u);
static_assert(!i2c_timing_for(3'000'000UL, I2cSpeed::standard_100k).has_value());

// The six columns (8.3.2.1).
static_assert(i2c1_pins_for(2).scl == Pad{'C', 16} && i2c1_pins_for(2).sda == Pad{'C', 17});
static_assert(i2c1_pins_for(4).scl == Pad{'C', 17} && i2c1_pins_for(6).sda == Pad{'C', 16});
static_assert(i2c1_pins_for(3).scl == Pad{'C', 19} && i2c1_pins_for(7).scl == Pad{'C', 18});
static_assert(i2c_pins_on_debug_port(i2c1_pins_for(3)) && i2c_pins_on_debug_port(i2c1_pins_for(5)));
static_assert(!i2c_pins_on_debug_port(i2c1_pins_for(2)) && i2c_pins_on_usb_pads(i2c1_pins_for(4)));
static_assert(!i2c_pins_valid(I2cPins{.scl = {'C', 16}, .sda = {'C', 16}, .remap = 2}));
static_assert(!i2c_pins_valid(I2cPins{.scl = {'C', 17}, .sda = {'C', 16}, .remap = 2}));

static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x48}));
static_assert(!i2c_address_config_valid(I2cAddressConfig{.own = 0x80}));
static_assert(!i2c_address_config_valid(
    I2cAddressConfig{.own = 0x123, .ten_bit = true, .second = 0x10}));
static_assert(i2c_address_config_valid(I2cAddressConfig{.own = 0x123, .ten_bit = true}));

} // namespace brio
