/*
 * usart.hpp
 *
 * The USART (RM ch. 14) in the two strata every brio serial driver
 * has (docs/design/serial.md):
 *
 *  Usart<n>              the RESOURCE: which instance, where its
 *                        registers are, its bus gate and reset, its
 *                        vector, its DMA channels (table 8-2), and the
 *                        whole register description of chapter 14 as
 *                        verbs - the frame, the divisor, mute mode with
 *                        both wakes, LIN's break, single-wire half
 *                        duplex, IrDA, the flow-control pair, the DMA
 *                        requests, every flag and every interrupt
 *                        enable;
 *  Uart<n, P, ...>       the TASK: the interrupt-driven byte transport
 *                        with two rings and one ISR body - every
 *                        console's personality, a ByteTransport for
 *                        print() and SerialPort, the same surface the
 *                        other three targets expose. Its OPTIONS ride
 *                        one trailing template parameter (single wire,
 *                        the flow-control pads, the frame) and cost
 *                        nothing when left at their defaults (measured:
 *                        every image byte-identical with the resource
 *                        and the options added).
 *
 * HALF DUPLEX IS A BUS HERE, NOT A LOOP. With HDSEL the receiver
 * listens on the TX pad through the alternate-function mux, and hears
 * a peer driving that wire - but not the instance's own frames: the
 * F1's internal loop-back is not what this silicon does (measured six
 * ways: push-pull or open drain, the RX pad released, floating, pulled
 * or handed to the function). The single-wire suite instrument the
 * STM32G0 stratum enjoys does not exist on this family; the serial
 * suite measures the transmitter on a timer capture and feeds the
 * receiver from a bit-banged pad instead.
 *
 * WHAT THIS CHAPTER HAS NOT GOT, by its register description and not by
 * its feature list: no synchronous mode (CTLR2 bits 11:7 are reserved
 * where the F1 keeps CLKEN/CPOL/CPHA/LBCL) and no smartcard (CTLR3 bits
 * 5:4 reserved where the F1 keeps SCEN/NACK, GPR with no guard time).
 * `has_synchronous` and `has_smartcard` say so at compile time, and
 * the family suite proves it on the silicon by writing those bits.
 *
 * TWO RINGS AND TWO FLAGS. Bytes leave through a TX ring drained by
 * the TXE interrupt (write_byte() re-arms TXEIE after a byte,
 * write_bulk() after a run's first byte and again after the rest; the
 * ISR disarms it when the ring runs dry, because TXE stands for ever
 * while the transmitter is idle and would re-enter the handler without
 * end), and arrive into an RX ring filled by RXNE, taken a byte at a
 * time by read_byte() or a run at a time, in place, by read_span() and
 * consume(). isr() returns the "RX went non-empty" EDGE that
 * util/serial_port.hpp wants to post on - one event per idle-to-busy
 * transition, not one per byte.
 *
 * THE BAUD DIVISOR IS THE WHOLE REGISTER. BRR counts the peripheral
 * clock's periods per bit in sixteenths (14.8.3: a 12-bit integer part
 * and a 4-bit fraction), so the value to store is simply pclk / baud -
 * no shifting, no separate fields - and actual_baud() inverts it,
 * which is how a program can report the error it is actually running
 * with. Below 16 the chapter's generator has nothing to divide by:
 * refused.
 *
 * ERRORS ARE READ THEN CLEARED. Framing, noise, parity and hardware
 * overrun stand in STATR and are cleared by reading STATR and then
 * DATAR, in that order (14.8.1) - so the handler reads the status
 * once, decides from that copy, and lets the DATAR read do the
 * clearing. A byte that arrived with an error is DROPPED rather than
 * pushed: a corrupted byte in a line assembler is worse than a gap,
 * and the counter says it happened.
 *
 * TWO OPTIONAL DMA ENGINE SLOTS, one for each of the instance's two
 * requests (14.6, DMAT and DMAR): a transmit engine drains the TX ring
 * by contiguous runs (the ring's read_span, consumed by exactly what
 * the block carried when it completes - the run started outside the
 * mask, the engine CLAIMED under it), and a receive engine runs in its
 * CIRCULAR SHAPE over the whole receive storage (ch32v00x/dma.hpp): the
 * channel writes it lap after lap and is never re-armed, its count is
 * the ring's producer index, and the receive ring is the consumer half
 * of that, util/ring.hpp's HardwareRing - so nothing is lost between
 * runs, there being none, and a lap the consumer did not keep up with is
 * counted and skipped (rx_overruns()). Without an engine the slot is the
 * NoDmaEngine tag and every engine branch is compiled out. An engine is
 * refused on any channel but the instance's own (table 8-2: USART1
 * transmits on channel 4 and receives on 5, USART2 on 6 and 7).
 *
 * UNDER THE RECEIVE ENGINE THE CPU NEVER READS DATAR, and the clear is
 * this silicon's - measured on the CH32V006K8U6 with frames banged into
 * USART2's receive pad and the channel reading DATAR: a read of STATR
 * ARMS the clear (14.8.1: "reading STATR and then DATAR"), and the next
 * read of DATAR - the channel's - clears every error flag and IDLE
 * standing at that read, including one its own frame just raised; a flag
 * with no status read since the last DATAR read stands. Clearing IDLE
 * that way also forgets the idle the clearing frame armed: one frame
 * after an idle that was read raises no IDLE of its own, two frames do.
 * (The CH32V203's USART answers the same; the STM32F4's, under the same
 * words, clears only what the status read saw.)
 *
 * THE BURST EDGE COMES FROM A VECTOR. With the receive engine the
 * USART's vector runs two states over that clear. WAITING FOR THE END:
 * IDLEIE, PEIE and EIE armed; an idle line or an error enters, its
 * status read counts the errors and arms the clear, the edge is
 * reported, and the vector turns to WAITING FOR A FRAME - those three
 * disarmed, RXNEIE armed (the channel still takes the byte; the
 * interrupt only says one came - measured, one entry a frame with RXNE
 * already clear and CNTR moved). That entry READS NO STATR, which would
 * arm the clear the next frame's own read performs, but CNTR alone: a
 * moved count turns the vector back, reporting the edge again - the
 * only edge a burst of one frame gets. The channel's half and full marks
 * report it from dma_isr(), so a stream with no silence is told twice a
 * lap. Two interrupts a burst, none a byte; the edge gated once per
 * idle-to-busy transition of the consumer, re-opened when its look finds
 * the ring empty. A CHANNEL THAT NEVER TAKES THE FRAME - stopped by a
 * transfer error, or frozen by a request another peripheral left on its
 * OR (the CH32V203's measured rule, docs/ch32vx03/dma.md) - leaves RXNE
 * standing under RXNEIE, and the wait would re-enter for ever: its
 * entries that find the count unmoved are counted, and the 255th gives
 * the channel up - a DMA fault counted, RXNEIE down, the edge reported,
 * nothing else touched in the vector - and the consumer's next look
 * restarts the ring from its first element and arms the wait for the
 * end, as after a transfer error.
 *
 * WHAT THE CHANNEL'S READ BOUNDS - the counts, never the bytes. The frame
 * after one whose error was counted loses its own (a run of errored
 * frames counts every other one, errors a clean frame apart count each);
 * so does the first frame of a burst after an idle the vector saw; and a
 * status read anywhere else - tx_idle(), the interrupt transmitter's
 * entry, a thread polling a flag - arms the clear for the next frame.
 *
 * THE PADS COME FROM THE REMAP TABLES (afio.hpp, tables 7-10 and
 * 7-11): the `remap` template parameter names a column, init() writes
 * it into AFIO_PCFR1, and the TX, RX, CTS and RTS pads follow. USART2's
 * default column puts TX on PA7, the CH32V006K8's reset pin, so USART2
 * is REFUSED at code 0 on this part and takes codes 1..6.
 *
 * THE MODES EXCLUDE EACH OTHER THE WAY 14.4 AND 14.5 SAY: half duplex
 * is refused while LIN or IrDA is on, IrDA while LIN or half duplex is
 * on or the stop bits are not one, LIN while either of the other two
 * is on - each verb refuses instead of storing a combination the
 * chapter declares undefined.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <atomic>
#include <span>

#include "ch32v00x/afio.hpp"
#include "ch32v00x/device.hpp"
#include "ch32v00x/dma_engine.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

/// The pads of an instance under a remap code (afio.hpp's tables):
/// USART1 at code 0 is TX PD5, RX PD6 - the pads the WCH-Link's own
/// serial is wired to on the bench board.
struct UsartPads {
    char tx_port;
    uint8_t tx_pin;
    char rx_port;
    uint8_t rx_pin;
};

constexpr UsartPads usart_pads_for(uint8_t instance, uint8_t remap = 0) {
    if (instance == 1) {
        const UsartPadSet p = afio_usart1_pads(remap);
        return UsartPads{p.tx.port, p.tx.pin, p.rx.port, p.rx.pin};
    }
    if (instance == 2) {
        const UsartPadSet p = afio_usart2_pads(remap);
        return UsartPads{p.tx.port, p.tx.pin, p.rx.port, p.rx.pin};
    }
    return UsartPads{'\0', 0, '\0', 0};
}

/// The whole column - TX, RX, CTS and RTS - under a remap code.
constexpr UsartPadSet usart_column_for(uint8_t instance, uint8_t remap = 0) {
    if (instance == 1) {
        return afio_usart1_pads(remap);
    }
    if (instance == 2) {
        return afio_usart2_pads(remap);
    }
    return UsartPadSet{};
}

constexpr uint32_t usart_base_for(uint8_t instance) {
    return instance == 1 ? pb2_base + 0x3800 : (instance == 2 && device::has_usart2) ? pb1_base + 0x4400 : 0;
}

/// BOTH gates are PB2's (3.4.7: USART2EN is bit 13 of RCC_PB2PCENR,
/// and USART2RST bit 13 of RCC_PB2PRSTR), although USART2's registers
/// answer in the PB1 address space - measured: gated on PB1's bit the
/// block reads zero in every register.
constexpr bool usart_on_pb2(uint8_t instance) { return instance == 1 || instance == 2; }
constexpr uint32_t usart_clock_for(uint8_t instance) {
    return instance == 1 ? rcc_pb2_usart1 : instance == 2 ? rcc_pb2_usart2 : 0;
}

constexpr Irq usart_irq_for(uint8_t instance) {
    return instance == 2 ? Irq::usart2 : Irq::usart1;
}

/// Table 8-2: the DMA channel that carries each instance's request.
constexpr uint8_t usart_dma_tx_channel(uint8_t instance) { return instance == 1 ? 4 : instance == 2 ? 6 : 0; }
constexpr uint8_t usart_dma_rx_channel(uint8_t instance) { return instance == 1 ? 5 : instance == 2 ? 7 : 0; }

/// Whether a remap code is legal for an instance on THIS PART: the
/// table's range, and USART2's default column refused because its TX
/// is the CH32V006K8's reset pin.
constexpr bool usart_remap_valid(uint8_t instance, uint8_t remap) {
    if (instance == 1) {
        return remap < afio_usart1_codes;
    }
    if (instance == 2) {
        return remap >= 1u && remap < afio_usart2_codes;
    }
    return false;
}

// =============================================================================
// The chapter's vocabulary
// =============================================================================

/// DATA bits of a frame. The register speaks in WORD length (M: 8 or
/// 9 bits including the parity bit when there is one), so seven data
/// bits exist only with a parity bit and nine only without.
enum class UartBits : uint8_t { seven = 7, eight = 8, nine = 9 };

enum class UartParity : uint8_t { none, even, odd };

/// CTLR2.STOP, all four codes; the half and one-and-a-half stops are
/// the chapter's own (14.8.5) and the receiver takes them as it takes
/// the others.
enum class UartStop : uint8_t { one = 0, half = 1, two = 2, one_and_half = 3 };

/// The frame the two ends agree on. Defaults to 8N1.
struct UartFormat {
    UartBits bits = UartBits::eight;
    UartParity parity = UartParity::none;
    UartStop stop = UartStop::one;
};

constexpr bool uart_format_valid(const UartFormat& f) {
    if (f.bits == UartBits::seven) {
        return f.parity != UartParity::none;
    }
    if (f.bits == UartBits::nine) {
        return f.parity == UartParity::none;
    }
    return true;
}

/// CTLR1's M, PCE and PS for a format (the word length is the data
/// bits plus the parity bit).
constexpr uint16_t usart_ctlr1_format(const UartFormat& f) {
    uint16_t v = 0;
    const bool parity = f.parity != UartParity::none;
    if (f.bits == UartBits::nine || (f.bits == UartBits::eight && parity)) {
        v |= usart_m;
    }
    if (parity) {
        v |= usart_pce;
    }
    if (f.parity == UartParity::odd) {
        v |= usart_ps;
    }
    return v;
}

constexpr uint16_t usart_ctlr2_stop(UartStop s) {
    return static_cast<uint16_t>(static_cast<uint16_t>(s) << usart_stop_shift);
}

/// The mask of the data bits a frame carries in DATAR (the parity bit,
/// when there is one, is the receiver's to check and not the byte's).
constexpr uint16_t uart_data_mask(const UartFormat& f) {
    return f.bits == UartBits::nine ? 0x1FFu : f.bits == UartBits::eight ? 0xFFu : 0x7Fu;
}

/// Mute mode (14.8.4 RWU/WAKE, 14.8.5 ADD): the receiver asleep until
/// the line goes idle, or until a frame whose MSB is set carries this
/// node's 4-bit address.
enum class MuteWake : uint8_t { idle_line, address_mark };

struct MuteConfig {
    MuteWake wake = MuteWake::idle_line;
    uint8_t address = 0;   ///< 0..15, for address_mark
};

constexpr bool mute_valid(const MuteConfig& c) { return c.address <= 15u; }

/// LIN mode (14.8.5): the break of thirteen low bits SBK sends, and
/// its detection at ten or eleven bits with its own flag and interrupt.
struct LinConfig {
    bool break_11bit = false;   ///< LBDL: 11-bit detection instead of 10
    bool break_interrupt = false;
};

/// IrDA (14.5, 14.8.6, 14.8.7): the SIR encoder on TX and decoder on RX,
/// normal mode at the bit rate (a 3/16 pulse) or low-power mode on the
/// prescaled clock. A prescaler of 0 "indicates retention" and is
/// refused.
struct IrdaConfig {
    bool low_power = false;
    uint8_t prescaler = 1;   ///< GPR.PSC
};

/// The synchronous mode's three choices (the CH32V003's 12.4, figure
/// 12-2): the clock's idle level (CPOL), the capture edge (CPHA) and
/// whether the last data bit gets a clock pulse too (LBCL).
struct UsartSyncConfig {
    bool clock_idle_high = false;
    bool capture_second_edge = false;
    bool last_bit_clock = false;
};

constexpr bool irda_valid(const IrdaConfig& c) { return c.prescaler != 0u; }

/// The divisor this clock and baud ask for: pclk/baud in sixteenths,
/// rounded to nearest so the error is halved. Below 16 there is no
/// whole clock period per sixteenth of a bit and the generator has
/// nothing to divide by - init() refuses such a value.
constexpr uint32_t usart_divisor(uint32_t pclk, uint32_t baud) {
    return baud == 0u ? 0u : (pclk + baud / 2u) / baud;
}

constexpr bool usart_divisor_valid(uint32_t brr) { return brr >= 16u && brr <= 0xFFFFu; }

// =============================================================================
// Usart<n>: the resource
// =============================================================================

/**
 * One USART instance, register by register. Every verb stores what it
 * names and nothing else; a verb whose combination the chapter
 * declares undefined refuses (false, nothing written). The task below
 * is written on these verbs; a program that wants the chapter beyond
 * the transport - a break on a LIN line, a muted receiver, an IrDA
 * pulse - reaches them here.
 */
template <uint8_t n>
struct Usart {
    static_assert(usart_base_for(n) != 0,
                  "brio Usart: this family has USART1 and USART2 (the CH32V005/006/007)");

    Usart() = delete;

    static constexpr uint8_t number = n;
    static constexpr Irq irq = usart_irq_for(n);
    static constexpr uint8_t dma_tx_channel = usart_dma_tx_channel(n);
    static constexpr uint8_t dma_rx_channel = usart_dma_rx_channel(n);

    /// Facts of the register description (the file header): the part's
    /// (parts/), stated so a program can ask at compile time and proven
    /// on the silicon by the suite. Where a part has them, their verbs
    /// arrive with its tier.
    static constexpr bool has_synchronous = device::usart_has_synchronous;
    static constexpr bool has_smartcard = device::usart_has_smartcard;

    static UsartRegs& regs() { return *reinterpret_cast<UsartRegs*>(usart_base_for(n)); }

    // ---- the block ----------------------------------------------------------
    static void bus_clock(bool on) {
        if constexpr (usart_on_pb2(n)) {
            if (on) { rcc()->PB2PCENR |= usart_clock_for(n); } else { rcc()->PB2PCENR &= ~usart_clock_for(n); }
        } else {
            if (on) { rcc()->PB1PCENR |= usart_clock_for(n); } else { rcc()->PB1PCENR &= ~usart_clock_for(n); }
        }
    }
    /// The RCC reset pulse: every register back at its reset value.
    static void reset() {
        if constexpr (usart_on_pb2(n)) {
            rcc()->PB2PRSTR |= usart_clock_for(n);
            rcc()->PB2PRSTR &= ~usart_clock_for(n);
        } else {
            rcc()->PB1PRSTR |= usart_clock_for(n);
            rcc()->PB1PRSTR &= ~usart_clock_for(n);
        }
    }
    static void remap(uint8_t code) {
        if constexpr (n == 1) {
            Afio::remap_usart1(code);
        } else {
            Afio::remap_usart2(code);
        }
    }

    static bool enabled() { return (regs().CTLR1 & usart_ue) != 0u; }
    /// UE. Off, "both the frequency divider and output will stop
    /// working after the current byte transfer is complete" (14.8.4).
    static void enable(bool on) { bit(regs().CTLR1, usart_ue, on); }
    static void transmitter(bool on) { bit(regs().CTLR1, usart_te, on); }
    static void receiver(bool on) { bit(regs().CTLR1, usart_re, on); }

    // ---- the frame and the rate -------------------------------------------
    /// Write the frame into CTLR1 (M, PCE, PS - the rest of the register
    /// kept) and CTLR2 (STOP), and the divisor into BRR. Refuses a
    /// format the word length cannot carry and a divisor below 16.
    static bool configure(const UartFormat& f, uint16_t brr) {
        if (!uart_format_valid(f) || !usart_divisor_valid(brr)) {
            return false;
        }
        UsartRegs& r = regs();
        r.CTLR1 = static_cast<uint16_t>((r.CTLR1 & ~(usart_m | usart_pce | usart_ps)) | usart_ctlr1_format(f));
        r.CTLR2 = static_cast<uint16_t>((r.CTLR2 & ~usart_stop_mask) | usart_ctlr2_stop(f.stop));
        r.BRR = brr;
        return true;
    }
    static bool stop_bits(UartStop s) {
        // 14.5: IrDA wants one stop bit.
        if ((regs().CTLR3 & usart_iren) != 0u && s != UartStop::one) {
            return false;
        }
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_stop_mask) | usart_ctlr2_stop(s));
        return true;
    }
    static bool set_brr(uint16_t v) {
        if (!usart_divisor_valid(v)) {
            return false;
        }
        regs().BRR = v;
        return true;
    }
    static uint16_t brr() { return regs().BRR; }
    /// The rate the divisor in the register gives at this clock.
    static uint32_t actual_baud(uint32_t pclk) {
        const uint32_t v = regs().BRR;
        return v == 0u ? 0u : pclk / v;
    }

    // ---- the modes ------------------------------------------------------------
    /// Mute mode: WAKE and ADD written, RWU left to mute()/unmute()
    /// because 14.8.4's note 1 says the receiver must have seen a byte
    /// before an idle-line wake can work.
    static bool mute_mode(const MuteConfig& c) {
        if (!mute_valid(c)) {
            return false;
        }
        bit(regs().CTLR1, usart_wake, c.wake == MuteWake::address_mark);
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_add_mask) | c.address);
        return true;
    }
    /// Put the receiver to sleep. 14.8.4 note 2: under an address-mark
    /// wake RWU cannot be set while RXNE stands - refused then.
    static bool mute() {
        if ((regs().CTLR1 & usart_wake) != 0u && (regs().STATR & usart_rxne) != 0u) {
            return false;
        }
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_rwu);
        return true;
    }
    static void unmute() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_rwu); }
    static bool muted() { return (regs().CTLR1 & usart_rwu) != 0u; }

    /// LIN mode. Refused while half duplex or IrDA is on (14.4, 14.5).
    static bool lin(const LinConfig& c) {
        if ((regs().CTLR3 & (usart_hdsel | usart_iren)) != 0u) {
            return false;
        }
        uint16_t v = static_cast<uint16_t>(regs().CTLR2 & ~(usart_lbdl | usart_lbdie));
        v |= usart_linen;
        if (c.break_11bit) { v |= usart_lbdl; }
        if (c.break_interrupt) { v |= usart_lbdie; }
        regs().CTLR2 = v;
        return true;
    }
    static void lin_off() {
        regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 & ~(usart_linen | usart_lbdl | usart_lbdie));
    }
    static bool lin_enabled() { return (regs().CTLR2 & usart_linen) != 0u; }
    /// SBK: one break frame after the current one; the bit clears
    /// itself when the break's stop bit is out. In LIN mode the break
    /// is thirteen bits long, elsewhere ten or eleven (14.2).
    static void send_break() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_sbk); }
    static bool break_pending() { return (regs().CTLR1 & usart_sbk) != 0u; }

    /// Single-wire half duplex (14.4): TX and RX joined inside the chip,
    /// the RX pad unused. Refused while LIN or IrDA is on.
    static bool half_duplex(bool on) {
        if (on && (regs().CTLR2 & usart_linen) != 0u) {
            return false;
        }
        if (on && (regs().CTLR3 & usart_iren) != 0u) {
            return false;
        }
        bit(regs().CTLR3, usart_hdsel, on);
        return true;
    }
    static bool half_duplex() { return (regs().CTLR3 & usart_hdsel) != 0u; }

    /// IrDA (14.5): refused while LIN or half duplex is on, with stop
    /// bits other than one, or with a prescaler of zero.
    static bool irda(const IrdaConfig& c) {
        if (!irda_valid(c)) {
            return false;
        }
        if ((regs().CTLR2 & usart_linen) != 0u || (regs().CTLR3 & usart_hdsel) != 0u) {
            return false;
        }
        if ((regs().CTLR2 & usart_stop_mask) != 0u) {
            return false;
        }
        regs().GPR = static_cast<uint16_t>((regs().GPR & ~usart_psc_mask) | c.prescaler);
        uint16_t v = static_cast<uint16_t>(regs().CTLR3 & ~usart_irlp);
        if (c.low_power) { v |= usart_irlp; }
        v |= usart_iren;
        regs().CTLR3 = v;
        return true;
    }
    static void irda_off() { regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 & ~(usart_iren | usart_irlp)); }
    static bool irda_enabled() { return (regs().CTLR3 & usart_iren) != 0u; }

    /// THE SYNCHRONOUS MODE (the CH32V003's 12.4): the CK pad of the
    /// column carries a clock while the transmitter shifts - and only
    /// then; the receiver samples on it. `has_synchronous` says whether
    /// the part has the bits: on the CH32V006 they are reserved and the
    /// verb answers false without a write. Refused while LIN, half
    /// duplex or IrDA is on (12.4 wants SCEN, HDSEL and IREN clear), and
    /// while the transmitter or the receiver is enabled - CPOL, CPHA
    /// and LBCL "need to be set when TE and RE are not enabled", so the
    /// caller pauses both around this verb. The CK pad itself is the
    /// caller's to hand to the peripheral (afio_usart1_pads(code).ck).
    static bool synchronous(const UsartSyncConfig& c) {
        if constexpr (!has_synchronous) {
            (void)c;
            return false;
        } else {
            if ((regs().CTLR2 & usart_linen) != 0u || (regs().CTLR3 & (usart_hdsel | usart_iren | usart_scen)) != 0u) {
                return false;
            }
            if ((regs().CTLR1 & (usart_te | usart_re)) != 0u) {
                return false;
            }
            uint16_t v = static_cast<uint16_t>(regs().CTLR2 & ~(usart_cpol | usart_cpha | usart_lbcl));
            if (c.clock_idle_high) { v |= usart_cpol; }
            if (c.capture_second_edge) { v |= usart_cpha; }
            if (c.last_bit_clock) { v |= usart_lbcl; }
            v |= usart_clken;
            regs().CTLR2 = v;
            return true;
        }
    }
    /// CLKEN and its three companions dropped; the same TE/RE rule.
    static bool synchronous_off() {
        if constexpr (!has_synchronous) {
            return false;
        } else {
            if ((regs().CTLR1 & (usart_te | usart_re)) != 0u) {
                return false;
            }
            regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 & ~(usart_clken | usart_cpol | usart_cpha | usart_lbcl));
            return true;
        }
    }
    static bool synchronous_enabled() {
        if constexpr (!has_synchronous) {
            return false;
        } else {
            return (regs().CTLR2 & usart_clken) != 0u;
        }
    }
    static uint8_t prescaler() { return static_cast<uint8_t>(regs().GPR & usart_psc_mask); }

    /// The hardware flow-control pair: RTS driven low while the receiver
    /// can take a frame, CTS sampled before each frame goes out.
    static void flow_control(bool rts, bool cts) {
        bit(regs().CTLR3, usart_rtse, rts);
        bit(regs().CTLR3, usart_ctse, cts);
    }
    static bool rts_enabled() { return (regs().CTLR3 & usart_rtse) != 0u; }
    static bool cts_enabled() { return (regs().CTLR3 & usart_ctse) != 0u; }

    static void dma_transmit(bool on) { bit(regs().CTLR3, usart_dmat, on); }
    static void dma_receive(bool on) { bit(regs().CTLR3, usart_dmar, on); }

    // ---- interrupts and flags --------------------------------------------
    /// CTLR1's five enables: usart_peie, usart_txeie, usart_tcie,
    /// usart_rxneie (ORE rides it), usart_idleie.
    static void interrupts(uint16_t ctlr1_mask, bool on) { bit(regs().CTLR1, ctlr1_mask, on); }
    static void rxne_interrupt(bool on) { bit(regs().CTLR1, usart_rxneie, on); }
    static void txe_interrupt(bool on) { bit(regs().CTLR1, usart_txeie, on); }
    static bool txe_interrupt() { return (regs().CTLR1 & usart_txeie) != 0u; }
    static void break_interrupt(bool on) { bit(regs().CTLR2, usart_lbdie, on); }
    static void cts_interrupt(bool on) { bit(regs().CTLR3, usart_ctsie, on); }
    /// EIE: FE, ORE and NE raise the vector - under DMAR only (14.8.6).
    static void error_interrupt(bool on) { bit(regs().CTLR3, usart_eie, on); }

    static uint16_t status() { return regs().STATR; }
    static bool flag(uint16_t mask) { return (regs().STATR & mask) != 0u; }
    /// The write-zero-to-clear flags (RXNE, TC, LBD, CTS); any other
    /// bit in the mask is ignored, because writing it does nothing.
    static void clear_flags(uint16_t mask) {
        regs().STATR = static_cast<uint16_t>(~(mask & usart_statr_rw0));
    }
    /// The read-sequence clear of IDLE, ORE, NE, FE and PE (and of RXNE,
    /// whose byte this discards).
    static void clear_by_read() {
        (void)regs().STATR;
        (void)regs().DATAR;
    }

    // ---- the data register --------------------------------------------------
    static bool tx_empty() { return (regs().STATR & usart_txe) != 0u; }
    static bool tx_complete() { return (regs().STATR & usart_tc) != 0u; }
    static bool rx_ready() { return (regs().STATR & usart_rxne) != 0u; }
    static void write_data(uint8_t b) { regs().DATAR = b; }
    /// A 9-bit word (M set): the ninth bit is data or, with parity on,
    /// the receiver's parity bit.
    static void write_word(uint16_t v) { regs().DATAR = static_cast<uint16_t>(v & 0x1FFu); }
    static uint8_t read_data() { return static_cast<uint8_t>(regs().DATAR & 0xFFu); }
    static uint16_t read_word() { return static_cast<uint16_t>(regs().DATAR & 0x1FFu); }
    static volatile uint16_t* data_address() { return &regs().DATAR; }

private:
    static void bit(volatile uint16_t& r, uint16_t mask, bool on) {
        r = static_cast<uint16_t>(on ? (r | mask) : (r & ~mask));
    }
};

// =============================================================================
// Uart: the transport task
// =============================================================================

/// What a Uart may be told beyond its rings and engines, as one
/// trailing template parameter. The defaults are the console's
/// personality and compile to exactly the code they did before this
/// struct existed.
struct UartOptions {
    /// The frame. The rings carry bytes, so nine data bits are refused
    /// here (the resource's word verbs speak them); seven with parity
    /// and eight with or without are fine.
    UartFormat format{};
    /// Single-wire half duplex (14.4): one pad, the TX pad, as an
    /// alternate-function OPEN DRAIN against an external pull-up - the
    /// receiver listens on that pad through the AF mux (measured: a
    /// plain input is not heard) and the RX pad is left alone. What
    /// the instance sends, its own receiver does NOT hear on this
    /// silicon (measured six ways), so this is a bus and not a
    /// loop-back.
    bool half_duplex = false;
    /// The flow-control pair on the column's CTS and RTS pads.
    bool rts = false;
    bool cts = false;
};

/**
 * The interrupt-driven serial port.
 *
 *   using Serial = brio::Uart<1>;
 *   constexpr Serial serial;                 // tag for print(serial, ...)
 *   Serial::init(clock, 115200);
 *   extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { Serial::isr(); }
 *
 * `P` is the platform, which the rings need to know whether an index
 * can be shared with a handler bare (atomic_width) or wants a guard.
 */
/**
 * The receive ring a transport keeps: a SkipRing, which the interrupt
 * receiver pushes into and tells when it lost a byte, without a receive
 * engine; with one, the CONSUMER HALF of
 * the ring the channel writes - util/ring.hpp's HardwareRing over a
 * storage array the transport owns (keyed by the transport's type), the
 * engine its RingCounter. A partial specialization, so that HardwareRing
 * over NoDmaEngine is never named.
 */
template <bool engine, typename Owner, uint16_t size, typename Engine, typename P>
struct UartRxRing {
    using type = SkipRing<uint8_t, size, P>;
};
template <typename Owner, uint16_t size, typename Engine, typename P>
struct UartRxRing<true, Owner, size, Engine, P> {
    static_assert(size >= 2u && size <= 0x8000u && (size & (size - 1u)) == 0u,
                  "brio Uart: a receive engine runs the whole receive ring as one circular "
                  "block - a power of two, 2..32768 bytes");
    static inline uint8_t storage[size]{};
    using type = HardwareRing<storage, Engine>;
};

template <uint8_t instance, typename P, uint16_t rx_size = 64, uint16_t tx_size = 64,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine, uint8_t remap = 0,
          UartOptions opts = {}>
struct Uart {
    static_assert(usart_base_for(instance) != 0,
                  "brio Uart: this family has USART1 and USART2 (the CH32V005/006/007)");
    static_assert(uart_format_valid(opts.format) && opts.format.bits != UartBits::nine,
                  "brio Uart: the rings carry bytes - seven data bits with a parity bit, or eight, "
                  "with or without one; nine-bit words are the resource's write_word()/read_word()");
    static_assert(!TxEngine::present || dma_engine_channel<TxEngine>() == usart_dma_tx_channel(instance),
                  "brio Uart: table 8-2 - this instance transmits on its own DMA channel "
                  "(USART1 on 4, USART2 on 6)");
    static_assert(!RxEngine::present || dma_engine_channel<RxEngine>() == usart_dma_rx_channel(instance),
                  "brio Uart: table 8-2 - this instance receives on its own DMA channel "
                  "(USART1 on 5, USART2 on 7)");
    static_assert(usart_remap_valid(instance, remap),
                  "brio Uart: no such remap code for this instance (afio.hpp's tables 7-10 and "
                  "7-11) - and USART2's default column is refused on the CH32V006K8, whose PA7 "
                  "is the reset pin: it takes codes 1..6");
    // An engine is checked where it is named: sizeof demands a complete
    // type, so an engine's own static_asserts fire on the application's
    // line.
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from ch32v00x/dma.hpp, or NoDmaEngine (the default)");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "the two engines of a Uart must not share a DMA channel");

    Uart() = default;   // a tag instance: constexpr Uart<1, P> serial;

    using Resource = Usart<instance>;

    static constexpr uint8_t number = instance;
    static constexpr uint8_t remap_code = remap;
    static constexpr UsartPads pads = usart_pads_for(instance, remap);
    static constexpr UsartPadSet column = usart_column_for(instance, remap);
    static constexpr UartOptions options = opts;
    static constexpr bool has_tx_engine = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;
    /// write_bulk()'s copy into the ring: the runtime's memcpy for a run of
    /// at least this many bytes whose source and ring slot share their
    /// alignment modulo the word (rt/rt.cpp's word path), the byte loop
    /// otherwise, where memcpy would run the same loop behind a call.
    static constexpr uint32_t copy_threshold = 16;

    using Tx = Pin<pads.tx_port, pads.tx_pin>;
    using Rx = Pin<pads.rx_port, pads.rx_pin>;
    using Cts = Pin<column.cts.port, column.cts.pin>;
    using Rts = Pin<column.rts.port, column.rts.pin>;

    static UsartRegs& regs() { return Resource::regs(); }

    /**
     * Open the port at `baud` in the options' frame (8N1 by default),
     * with the receive interrupt armed.
     *
     * Returns false when the divisor comes out below the smallest the
     * chapter allows (16, i.e. one whole peripheral-clock period per
     * bit sixteenth): a baud rate this clock cannot serve is a fact the
     * caller must see rather than a port that mangles every frame.
     */
    template <typename C>
    static bool init(C clock, uint32_t baud) {
        static_assert(clock_follows<C, Uart>(),
                      "brio Uart: the baud divisor is derived from the peripheral "
                      "clock, so a dynamic clock must list this port among the users "
                      "it rebases");

        const uint32_t pclk = clock_hz(clock);
        const uint32_t brr = divisor_for(pclk, baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }

        Resource::bus_clock(true);
        // The block through its reset line first: whatever the program or
        // an earlier transport on this instance left - a status read that
        // armed a clear, a request bit, an enable - is gone, and the words
        // below are the whole configuration.
        Resource::reset();
        Resource::remap(remap);

        // Pads before the enable: TE's idle frame must land on a pad the
        // peripheral already owns.
        if constexpr (opts.half_duplex) {
            Tx::function(PinDrive::open_drain);   // the one pad; RX is left alone
        } else {
            Tx::function();                  // alternate function, push-pull
            Rx::input();                     // floating: the peer drives it
        }
        if constexpr (opts.rts) {
            Rts::function();
        }
        if constexpr (opts.cts) {
            Cts::input(PinPull::up);         // unconnected reads "not clear to send"
        }

        // The registers whole, with the port disabled: CTLR1 carries the
        // frame alone until the enable below, CTLR2 the stop bits, CTLR3
        // the options and the DMA requests of the engines named.
        regs().CTLR1 = usart_ctlr1_format(opts.format);
        regs().CTLR2 = usart_ctlr2_stop(opts.format.stop);
        uint16_t ctlr3 = 0;
        if constexpr (has_tx_engine) { ctlr3 |= usart_dmat; }
        if constexpr (has_rx_engine) { ctlr3 |= usart_dmar; }
        if constexpr (opts.half_duplex) { ctlr3 |= usart_hdsel; }
        if constexpr (opts.rts) { ctlr3 |= usart_rtse; }
        if constexpr (opts.cts) { ctlr3 |= usart_ctse; }
        regs().CTLR3 = ctlr3;
        regs().BRR = static_cast<uint16_t>(brr);
        // RXNE is the receive channel's when an engine has it, and the
        // vector then waits for the end of a burst: the idle line and the
        // errors (EIE under DMAR, PE on its own enable).
        if constexpr (has_rx_engine) {
            regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 | usart_eie);
        }
        regs().CTLR1 = static_cast<uint16_t>(usart_ctlr1_format(opts.format) | usart_ue | usart_te | usart_re |
                                             (has_rx_engine ? static_cast<uint16_t>(usart_idleie | usart_peie)
                                                            : usart_rxneie));
        m_baud = baud;

        m_tx.clear();
        m_rx.clear();
        clear_errors();
        m_dma_faults = 0;

        if constexpr (has_rx_engine) {
            m_rx_waiting = false;
            m_rx_drained = true;
            // The whole receive storage, circular, its half and full
            // marks the edge of a stream with no silence.
            if (!RxEngine::arm_ring(&regs().DATAR, RxRing::storage, true)) {
                return false;
            }
        }
        if constexpr (has_tx_engine) {
            TxEngine::arm(&regs().DATAR);
        }

        Pfic::enable(usart_irq_for(instance));
        return true;
    }

    /**
     * The ISR body of WHICHEVER DMA channel this transport owns - call
     * it from the vector(s) the engines' channels report on:
     *
     *     extern "C" BRIO_CH32_INTERRUPT void dma1_channel4_handler() { (void)Serial::dma_isr(); }
     *
     * The flags of every channel are one register (INTFR), read ONCE
     * here for both engines; each engine takes and clears only its own
     * channel's armed flags, so this is safe on a vector another channel
     * of the program shares nothing with. On the transmit channel a
     * completion releases exactly the block's bytes from the ring and
     * starts the next run; on the receive channel a completion is a LAP
     * of the ring, counted, and the lap's half and full marks are the
     * receive edge; a transfer error stops the channel, and the
     * consumer's next look starts it again.
     *
     * Returns true when the receive ring holds bytes its consumer has not
     * been told of - the edge, as isr()'s: post RxActivity on true.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        bool edge = false;
        uint32_t intfr = 0;
        if constexpr (has_tx_engine) {
            intfr = TxEngine::block_flags();
        } else if constexpr (has_rx_engine) {
            intfr = RxEngine::block_flags();
        }
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service(intfr);
            if ((f & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                m_dma_faults = m_dma_faults + 1u;
            } else if ((f & TxEngine::flag_complete) != 0u) {
                m_tx.consume(static_cast<typename decltype(m_tx)::index_t>(TxEngine::complete()));
                pump_tx();
            }
        }
        if constexpr (has_rx_engine) {
            const uint8_t f = RxEngine::service(intfr);
            if ((f & RxEngine::flag_error) != 0u) {
                (void)RxEngine::abandon();
                m_dma_faults = m_dma_faults + 1u;
                m_rx_drained = false;
                edge = true;   // the consumer must come: its look starts the ring again
            } else if (f != 0u) {
                if ((f & RxEngine::flag_complete) != 0u) {
                    RxEngine::lap();   // a lap: counted, nothing re-armed
                }
                edge = told();   // the half or the full mark of a lap
            }
        }
        return edge;
    }

    /**
     * The receive edge asked from the consumer's side, and the ring's
     * housekeeping. NOBODY NEEDS TO ASK: isr() and dma_isr() report the
     * edge (the file header). This is the same gate from the main
     * context, for an owner that still asks - true when the ring holds
     * bytes and the consumer has found it empty since the last true, from
     * a vector or from here - and a channel a transfer error stopped is
     * started again (the look is the consumer's, so is the restart; every
     * read verb does the same). It reads neither STATR nor DATAR: the
     * errors are the vector's to count. False, and free, without an
     * engine.
     */
    static bool harvest() {
        if constexpr (!has_rx_engine) {
            return false;
        } else {
            restart_if_stopped();
            // waiting(): a look that writes nothing - empty() may skip,
            // moving the tail under a run the consumer holds.
            if (!m_rx_drained || m_rx.waiting() == 0u) {
                return false;
            }
            m_rx_drained = false;
            return true;
        }
    }

    /// Transfer errors the engines reported (a fault is a block thrown
    /// away), and receive channels given up on a frame they never took.
    /// Always 0, and free, without an engine.
    static uint16_t dma_faults() { return m_dma_faults; }

    /// The divisor this clock and baud ask for (usart_divisor()).
    static constexpr uint32_t divisor_for(uint32_t pclk, uint32_t baud) { return usart_divisor(pclk, baud); }

    /// The baud the port is ACTUALLY running at, from the divisor in the
    /// register - what a program reports instead of what it asked for.
    static uint32_t actual_baud(uint32_t pclk) { return Resource::actual_baud(pclk); }

    /**
     * The port's whole interrupt body - bind the USART1 vector to it.
     *
     * Returns true when a byte arrived into an EMPTY ring: the edge
     * util/serial_port.hpp posts on.
     */
    // always_inline: one call site (the app's vector binding), so the
    // compiler saves only the registers this body uses.
    [[gnu::always_inline]] static bool isr() {
        bool rx_edge = false;
        bool waited = false;
        // With a receive engine the handler never reads DATAR, and the
        // wait for a frame comes first: it reads no STATR, and with a
        // transmit engine nothing else of this port raises the vector.
        if constexpr (has_rx_engine) {
            if (m_rx_waiting) {
                waited = true;
                rx_edge = rx_frame_entry();
                if constexpr (has_tx_engine) {
                    return rx_edge;
                }
            }
        }

        const uint16_t status = regs().STATR;

        if constexpr (has_rx_engine) {
            if (!waited) {
                rx_edge = rx_end_entry(status);
            }
        } else if ((status & usart_rxne) != 0u) {
            const uint8_t byte = static_cast<uint8_t>(regs().DATAR & 0xFFu);
            // Every byte lost is reported to the receive ring (lost()):
            // the flagged byte dropped here - with an overrun's, lost in
            // the shift register - and one a full ring refuses. The
            // consumer's next look skips everything queued.
            if ((status & (usart_fe | usart_ne | usart_pe | usart_ore)) != 0u) {
                if ((status & usart_fe) != 0u) { bump(m_frame_errors); }
                if ((status & usart_ne) != 0u) { bump(m_noise_errors); }
                if ((status & usart_pe) != 0u) { bump(m_parity_errors); }
                if ((status & usart_ore) != 0u) { bump(m_hw_overruns); }
                m_rx.lost();
            } else {
                const bool was_empty = m_rx.empty();
                if (m_rx.push(byte)) {
                    rx_edge = was_empty;
                } else {
                    bump(m_rx_overruns);
                    m_rx.lost();
                }
            }
        }

        if (!has_tx_engine && (status & usart_txe) != 0u && (regs().CTLR1 & usart_txeie) != 0u) {
            if (const auto next = m_tx.pop()) {
                regs().DATAR = *next;
            } else {
                // TXE stands while the transmitter is idle: disarm, or
                // the handler is re-entered for ever.
                regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_txeie);
            }
        }

        return rx_edge;
    }

    // ---- byte transport (ByteSink / ByteSource) ---------------------------

    /// Queue one byte; false when the TX ring is full (print() spins).
    /// Without an engine TXE is armed; with one the engine is nudged -
    /// and a REFUSED byte nudges too, since the ring is full exactly
    /// when nothing is draining it and print() would otherwise spin on
    /// a transport nobody poked.
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            if constexpr (has_tx_engine) {
                pump_tx();
            }
            return false;
        }
        if constexpr (has_tx_engine) {
            pump_tx();
        } else {
            arm_txe();
        }
        return true;
    }

    /// Queue a RUN (util/stream.hpp's BulkSink): as much of `src` as the
    /// ring has room for, copied into the ring's own free run and handed
    /// over with one index store per contiguous part. Never blocks;
    /// returns how many were queued. A run that finds no room is
    /// write_byte()'s refusal: nothing armed on the plain transport (the
    /// push that filled the ring armed TXEIE, a level), the engine nudged
    /// on the engined one.
    ///
    /// THE FIRST BYTE GOES BEFORE THE COPY, on the plain transport: it is
    /// pushed and TXEIE armed exactly as write_byte() does, so an idle
    /// transmitter starts on it while the rest is copied. The handler
    /// disarms on the TXE entry that finds the ring empty - the one after
    /// it hands DATAR that byte, TXE rising again as the byte moves to the
    /// shift register - and the first byte was the whole ring, so the
    /// rest, once published, is armed again: one more CTLR1
    /// read-modify-write, idempotent, two a run at most. WITH AN ENGINE
    /// the run is copied whole and the engine nudged once, a block being
    /// what it starts on.
    static uint32_t write_bulk(std::span<const uint8_t> src) {
        uint32_t queued = 0;
        if constexpr (!has_tx_engine) {
            // write_byte()'s two steps, spelled here so the run's path
            // holds no call: a full ring is its refusal, nothing to arm.
            if (src.empty() || !m_tx.push(src[0])) {
                return 0;
            }
            arm_txe();
            queued = 1;
        }
        while (queued < src.size()) {
            const auto room = m_tx.write_span();
            if (room.empty()) {
                break;
            }
            const uint32_t want = static_cast<uint32_t>(src.size()) - queued;
            const uint32_t take = want < room.size() ? want : static_cast<uint32_t>(room.size());
            // Two pointers and no index, and the test at the bottom: a
            // load, a store, two steps and one branch a byte, where an index
            // re-adds both bases every byte. `take` is at least one here -
            // the room is not empty and the run is not done.
            const uint8_t* from = src.data() + queued;
            uint8_t* to = room.data();
            if (take >= copy_threshold &&
                ((reinterpret_cast<uintptr_t>(to) ^ reinterpret_cast<uintptr_t>(from)) & 3u) == 0u) {
                (void)memcpy(to, from, take);
            } else {
                uint8_t* const end = to + take;
                do {
                    *to++ = *from++;
                } while (to != end);
            }
            m_tx.publish(static_cast<typename decltype(m_tx)::index_t>(take));
            queued += take;
        }
        if constexpr (has_tx_engine) {
            pump_tx();
        } else if (queued > 1u) {
            // The rest, behind a first byte the handler may have disarmed on.
            arm_txe();
        }
        return queued;
    }

    /// Take one received byte; false when none is pending - or, with a
    /// receive engine, when it was written over while it was read
    /// (rx_overruns() counts it).
    static bool read_byte(uint8_t& b) {
        if constexpr (has_rx_engine) {
            const std::span<const uint8_t> run = look();
            if (run.empty()) {
                return false;
            }
            b = run[0];
            return m_rx.consume(1u);
        } else {
            const auto v = m_rx.pop();
            if (!v) {
                return false;
            }
            b = *v;
            return true;
        }
    }

    /// The received bytes IN PLACE: the contiguous run ready to be read,
    /// never wrapping - the receive ring's consumer half under the ring's
    /// own names. With consume() it is util/stream.hpp's SpanSource,
    /// which SerialPort drains a run at a time.
    /// With a receive engine the run is the CHANNEL'S ring, read where it
    /// writes, and consume() says whether it was intact.
    static std::span<const uint8_t> read_span() {
        if constexpr (has_rx_engine) {
            return look();
        } else {
            return m_rx.read_span();
        }
    }

    /// Release the first `count` bytes of read_span(), oldest first, clamped
    /// to what is queued. With a receive engine it answers whether the run
    /// was intact when it was read - false when the channel wrote over it
    /// while it was held, which the ring counts in rx_overruns() and skips.
    static auto consume(uint32_t count) {
        if constexpr (has_rx_engine) {
            return m_rx.consume(count);
        } else {
            constexpr uint32_t most = decltype(m_rx)::capacity();
            m_rx.consume(static_cast<typename decltype(m_rx)::index_t>(count < most ? count : most));
        }
    }

    // ---- introspection ----------------------------------------------------

    static auto rx_pending() { return m_rx.count(); }

    /// The gaps in the stream the receive ring hands out, since the
    /// program started, NEVER CLEARED - util/stream.hpp's SkippingSource,
    /// util/serial_port.hpp's epoch. Without an engine: the SkipRing's
    /// skips (modulo 2^8), one at the consumer's look after a byte the
    /// handler dropped for a flag or a full ring, or an overrun - the look
    /// that discards what the ring held. With one: the view's skips (a lap missed, a held run refused) plus
    /// m_rx_lost - an overrun, a restart after a transfer error (a framed
    /// frame is stored: the channel moves it) - counted when the vector or
    /// the look sees them, so the line torn is the one begun when the
    /// consumer next looks.
    static uint32_t rx_skips() {
        if constexpr (has_rx_engine) {
            return m_rx.skips() + m_rx_lost;
        } else {
            return m_rx.skips();
        }
    }

    /// THE WIRE IS IDLE: nothing queued, no block in flight, and the last
    /// frame's stop bit out (STATR.TC, 14.8.1). A transmit block clears TC
    /// as it starts: the channel's writes of DATAR run no part of TC's
    /// software clear, and the flag would answer from the frame before the
    /// block.
    static bool tx_idle() {
        if constexpr (has_tx_engine) {
            if (TxEngine::busy()) {
                return false;
            }
        }
        return m_tx.empty() && (regs().STATR & usart_tc) != 0u;
    }

    /// Bytes the receive side lost to a consumer that did not keep up:
    /// without an engine, a byte a full ring refused; with one, a lap of
    /// the channel's ring written over unread bytes, or a run written over
    /// while held - each one a skip, saturating here.
    static uint16_t rx_overruns() {
        if constexpr (has_rx_engine) {
            const uint32_t n = m_rx.overruns();
            return n > 0xFFFFu ? uint16_t{0xFFFF} : static_cast<uint16_t>(n);
        } else {
            return m_rx_overruns;
        }
    }
    static uint16_t frame_errors() { return m_frame_errors; }
    static uint16_t parity_errors() { return m_parity_errors; }
    static uint16_t noise_errors() { return m_noise_errors; }
    static uint16_t hw_overruns() { return m_hw_overruns; }

    static void clear_errors() {
        if constexpr (has_rx_engine) {
            m_rx.clear_overruns();
        }
        m_rx_overruns = 0;
        m_frame_errors = 0;
        m_parity_errors = 0;
        m_noise_errors = 0;
        m_hw_overruns = 0;
    }

    /// Follow a clock that changed rate. Drains what is in flight first:
    /// a byte half out at the old rate would finish at the new one.
    static void rebase(uint32_t hz) {
        drain();
        const uint32_t baud = actual_baud_cached();
        const uint32_t brr = divisor_for(hz, baud);
        if (usart_divisor_valid(brr)) {
            regs().BRR = static_cast<uint16_t>(brr);
        }
    }

    /// Move the LINK to a different bit rate, the clock staying put - the
    /// mirror of rebase(), `hz` the peripheral clock as rebase() takes it
    /// (HCLK: this family has no bus prescaler on the USARTs' path, 14.3).
    /// Drains what is in flight at the old rate first, bounded. False, and
    /// nothing written, when the new rate is unreachable.
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const uint32_t brr = divisor_for(hz, baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }
        drain();
        regs().BRR = static_cast<uint16_t>(brr);
        m_baud = baud;
        return true;
    }

    /// Whether `baud` is reachable from a peripheral clock `pclk`, and the
    /// smallest clock that can produce it: sixteen clocks a bit (14.3).
    static constexpr bool can_baud(uint32_t pclk, uint32_t baud) {
        return usart_divisor_valid(usart_divisor(pclk, baud));
    }
    static constexpr uint32_t min_hz_for(uint32_t baud) { return baud * 16u; }

    /// Stop the port and park its pads: the vector off, the engines'
    /// channels stopped, UE clear, the block's reset pulsed, the gate
    /// closed, the pads released.
    ///
    /// THE RESET LINE BEFORE THE GATE. A channel of this controller is an
    /// OR of the requests wired to it (RM table 8-2: USART2 shares 6 and 7
    /// with I2C1, USART1's receive shares 5 with TIM1's update), and on the
    /// CH32V203 and the CH32V303 - the same controller's lineage - a
    /// request once raised is HELD until the channel acknowledges it or the
    /// block is reset: clearing DMAT, DMAR and UE does not withdraw it, and
    /// a gated clock freezes it on the OR, where the next owner's first
    /// enable moves one item and the channel stalls
    /// (docs/ch32vx03/dma.md). Whether this die holds one is not measured
    /// (docs/ch32v00x/usart.md); the pulse - the vendor's own USART_DeInit,
    /// two read-modify-writes of the bus's reset register - leaves none
    /// either way, and takes CTLR3 with it.
    static void release() {
        Pfic::disable(usart_irq_for(instance));
        if constexpr (has_tx_engine) {
            TxEngine::stop();
        }
        if constexpr (has_rx_engine) {
            RxEngine::stop();
        }
        regs().CTLR1 = 0;
        Resource::reset();
        Resource::bus_clock(false);
        Tx::release();
        if constexpr (!opts.half_duplex) {
            Rx::release();
        }
        if constexpr (opts.rts) {
            Rts::release();
        }
        if constexpr (opts.cts) {
            Cts::release();
        }
    }

private:
    /// Start the next contiguous run of the TX ring on the engine, if it
    /// is idle and there is one. THE MASK COVERS THE CLAIM AND NOTHING
    /// ELSE: the engine's busy flag tested and set under the guard - the
    /// completion path, which clears it, runs in the channel's handler -
    /// and the run read and the block programmed outside it, because a
    /// claimed engine has no block in flight and so no completion can
    /// land under the programming, nor any consume() move the ring's
    /// tail. The producer is this context (the ring is SPSC), so a run
    /// found empty after the claim stays empty until the claim is given
    /// back.
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            if (m_tx.empty()) {
                return;
            }
            {
                typename P::CriticalSection cs;
                if (!TxEngine::claim()) {
                    return;
                }
            }
            const auto run = m_tx.read_span();
            if (run.empty()) {
                TxEngine::unclaim();
                return;
            }
            Resource::clear_flags(usart_tc);   // TC from the frame before this block, written 0
            (void)TxEngine::launch(run);
        }
    }

    /// The transmit ring out to the wire, bounded: what rebase() and
    /// set_baud() wait for before the divisor moves.
    static void drain() {
        constexpr uint32_t drain_spins = 2'000'000UL;
        uint32_t spins = drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
            if constexpr (has_tx_engine) {
                pump_tx();
            }
        }
        spins = drain_spins;
        while ((regs().STATR & usart_tc) == 0u && spins-- != 0u) {
        }
    }

    /// TXEIE armed from the thread. With a receive engine and no transmit
    /// one the vector rewrites CTLR1's receive enables, so the thread's
    /// read-modify-write takes the guard - the decision, three
    /// instructions - or it could store back an enable the vector had just
    /// turned; every other shape stores bare.
    [[gnu::always_inline]] static void arm_txe() {
        if constexpr (has_rx_engine) {
            typename P::CriticalSection cs;
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_txeie);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_txeie);
        }
    }

    /// The consumer's look at the receive ring under an engine: a channel
    /// a transfer error stopped started again first, and a look that finds
    /// the ring empty RE-OPENS THE EDGE and looks again, so a byte landing
    /// between the two is in this run or raises the edge.
    static std::span<const uint8_t> look() {
        restart_if_stopped();
        std::span<const uint8_t> run = m_rx.read_span();
        if (run.empty()) {
            m_rx_drained = true;
            std::atomic_signal_fence(std::memory_order_seq_cst);
            run = m_rx.read_span();
        }
        return run;
    }

    /// A ring never stops on its own: a channel that is not running was
    /// stopped by a transfer error, and is bound again from the storage's
    /// first element with the view - the consumer's context.
    ///
    /// A ring the vector GAVE UP on (rx_stalled()) is bound again the
    /// same way: its channel still enabled over a frozen handshake, the
    /// binding's first store takes EN down.
    ///
    /// A RESTARTED RING WAITS FOR THE END: the vector's state is put back
    /// with the channel - the wait for a frame it was in counted the old
    /// lap's position, and a channel the vector gave up on left every
    /// receive enable down. One read-modify-write of each control register
    /// under the guard, the transmitter's TXEIE sharing CTLR1 with them;
    /// the rare path only.
    static void restart_if_stopped() {
        if constexpr (has_rx_engine) {
            if (RxEngine::idle() || m_rx_given_up) {
                m_rx_given_up = false;
                (void)RxEngine::arm_ring(&regs().DATAR, RxRing::storage, true);
                m_rx.clear();
                m_rx_lost = m_rx_lost + 1u;   // the unread bytes went with the stopped lap
                typename P::CriticalSection cs;
                m_rx_waiting = false;
                regs().CTLR1 = static_cast<uint16_t>((regs().CTLR1 & ~usart_rxneie) |
                                                     usart_idleie | usart_peie);
                regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 | usart_eie);
            }
        }
    }

    /// THE EDGE'S GATE, for the vectors: true once per idle-to-busy
    /// transition of the consumer. Each caller has seen the channel write
    /// since the consumer was last told, so no look at the view is needed.
    [[gnu::always_inline]] static bool told() {
        if (m_rx_drained) {
            m_rx_drained = false;
            return true;
        }
        return false;
    }

    /// WAITING FOR THE END (the file header): an idle line or an error,
    /// from the entry's one status read. An entry showing neither and no
    /// transmit condition to explain it is an idle line a status read
    /// elsewhere armed and a frame cleared before this vector ran: served
    /// as the idle it was. CNTR is read before RXNEIE is armed and after:
    /// a frame the channel took between raises no entry, so it is served
    /// here.
    [[gnu::always_inline]] static bool rx_end_entry(uint16_t status) {
        if ((status & (usart_idle | usart_fe | usart_ne | usart_pe | usart_ore)) == 0u) {
            if constexpr (!has_tx_engine) {
                if ((status & usart_txe) != 0u && (regs().CTLR1 & usart_txeie) != 0u) {
                    return false;   // the transmitter's entry
                }
            }
        }
        if ((status & usart_fe) != 0u) { bump(m_frame_errors); }
        if ((status & usart_ne) != 0u) { bump(m_noise_errors); }
        if ((status & usart_pe) != 0u) { bump(m_parity_errors); }
        if ((status & usart_ore) != 0u) {
            bump(m_hw_overruns);
            m_rx_lost = m_rx_lost + 1u;
        }
        m_rx_waiting = true;
        const uint16_t at = static_cast<uint16_t>(RxEngine::remaining());
        m_rx_at = at;
        m_rx_unmoved = 0;
        regs().CTLR1 = static_cast<uint16_t>((regs().CTLR1 & ~(usart_idleie | usart_peie)) | usart_rxneie);
        regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 & ~usart_eie);
        const bool edge = told();
        if (static_cast<uint16_t>(RxEngine::remaining()) == at) {
            return edge;
        }
        const bool first = rx_frame_entry();
        return edge || first;
    }

    /// WAITING FOR A FRAME: the channel has moved since the wait began, so
    /// its read finished the clear - back to waiting for the end, and the
    /// edge, which is the only one a burst of one frame gets. NO STATR
    /// READ: it would arm the clear the next frame's read performs.
    ///
    /// An entry with the count unmoved is the transmitter's, or a frame
    /// the channel has not taken yet - or one it NEVER takes: RXNE is a
    /// level, and over a channel that does not serve its request (a
    /// transfer error stopped it, or another peripheral's held request
    /// froze its handshake) this vector would be re-entered for ever, the
    /// thread starved. So the entries that find the count unmoved with
    /// nothing else armed to explain them are COUNTED, and the
    /// rx_stall_entries-th gives the channel up (rx_stalled()); the count
    /// then stays there, the state parked until the consumer's look
    /// restarts the ring. Two compares and an add an entry, on this path
    /// alone.
    [[gnu::always_inline]] static bool rx_frame_entry() {
        if (static_cast<uint16_t>(RxEngine::remaining()) == m_rx_at) {
            if constexpr (!has_tx_engine) {
                if ((regs().CTLR1 & usart_txeie) != 0u) {
                    return false;   // the transmitter's entry, perhaps
                }
            }
            if (m_rx_unmoved == rx_stall_entries) {
                return false;   // given up already: the consumer's look restarts it
            }
            m_rx_unmoved = static_cast<uint8_t>(m_rx_unmoved + 1u);
            if (m_rx_unmoved != rx_stall_entries) {
                return false;   // the frame not yet taken
            }
            return rx_stalled();
        }
        m_rx_waiting = false;
        regs().CTLR1 = static_cast<uint16_t>((regs().CTLR1 & ~usart_rxneie) | usart_idleie | usart_peie);
        regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 | usart_eie);
        return told();
    }

    /// How many entries of the wait for a frame may find the channel's
    /// count unmoved before the channel is given up. A live channel takes
    /// RXNE's byte within its arbitration - a few bus cycles behind any
    /// other channel's item - where one entry of this vector is tens of
    /// cycles, so a frame not yet taken is one or two entries; 255 is
    /// thousands of cycles of a channel that moved nothing. A channel a
    /// higher-priority memory-to-memory block starves for that long is
    /// given up too, and restarted by the consumer's next look.
    static constexpr uint8_t rx_stall_entries = 255;

    /// THE CHANNEL GIVEN UP: the fault counted, the ring marked for the
    /// consumer's look to bind again (restart_if_stopped(), which stops
    /// the channel and starts it from the storage's first element, as
    /// after a transfer error) and the edge reported so the consumer comes.
    /// RXNEIE goes down, the one enable of this state (IDLEIE, PEIE and EIE
    /// are already down), so the standing RXNE re-enters nothing in
    /// between; the state stays the wait for a frame with its count at the
    /// bound, parked. The channel itself is left to the thread, so the
    /// vector stays the few stores it is.
    [[gnu::always_inline]] static bool rx_stalled() {
        bump(m_dma_faults);
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_rxneie);
        m_rx_given_up = true;
        m_rx_drained = false;
        return true;
    }

    /// Saturating: a counter that wrapped would understate the damage.
    [[gnu::always_inline]] static void bump(uint16_t& counter) {
        if (counter != 0xFFFFu) {
            ++counter;
        }
    }

    static uint32_t actual_baud_cached() { return m_baud; }

    using RxRing = UartRxRing<RxEngine::present, Uart, rx_size, RxEngine, P>;
    static inline typename RxRing::type m_rx{};
    static inline Ring<uint8_t, tx_size, P> m_tx;
    /// Whether the consumer has found the receive ring empty since the edge
    /// was last reported - set by its look, cleared by whoever reports.
    static inline volatile bool m_rx_drained = true;
    /// The receive engine's vector state: waiting for a frame, and CNTR
    /// when that wait began.
    static inline volatile bool m_rx_waiting = false;
    static inline volatile uint16_t m_rx_at = 0;
    /// The entries of that wait that found the count unmoved
    /// (rx_frame_entry()); the vector's alone.
    static inline uint8_t m_rx_unmoved = 0;
    /// The vector gave the channel up: set there, cleared by the
    /// consumer's restart.
    static inline volatile bool m_rx_given_up = false;
    static inline uint32_t m_baud = 0;

    static inline uint16_t m_rx_overruns = 0;
    static inline uint16_t m_frame_errors = 0;
    static inline uint16_t m_parity_errors = 0;
    static inline uint16_t m_noise_errors = 0;
    static inline uint16_t m_hw_overruns = 0;
    static inline uint16_t m_dma_faults = 0;
    /// Under a receive engine, the bytes lost beside the view's skips (an
    /// overrun, a restart), never cleared: rx_skips()'s second term.
    static inline volatile uint32_t m_rx_lost = 0;
};

} // namespace brio
