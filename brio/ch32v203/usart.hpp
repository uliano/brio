/*
 * usart.hpp
 *
 * The CH32V203's serial ports (RM ch. 18) in the two strata every brio
 * serial driver has (docs/design/serial.md):
 *
 *  Usart<n>              the RESOURCE: which instance, where its
 *                        registers are, its bus gate and its reset, its
 *                        vector, its DMA channels, and the whole
 *                        register description of chapter 18 as verbs -
 *                        the frame, the divisor, mute mode with both
 *                        wakes, LIN's break, single-wire half duplex,
 *                        IrDA, the smartcard, the synchronous clock,
 *                        the flow-control pair, the DMA requests, every
 *                        flag and every interrupt enable;
 *  Uart<n, P, ...>       the TASK: the interrupt-driven byte transport
 *                        with two rings and one ISR body - every
 *                        console's personality, a ByteTransport for
 *                        print() and SerialPort, the same surface the
 *                        other five targets expose.
 *
 * FOUR INSTANCES, TWO BUSES. USART1 sits on PB2 and USART2, USART3 and
 * UART4 on PB1 - which on this family do NOT run at the same rate above
 * 72 MHz (clock.hpp caps PB1 there), so the transport asks its own bus
 * for the clock its divisor counts. WHICH instances a part offers is the
 * part's table (device::has_usart), and the list is not always the first
 * n of them: the smallest part offers one usart and it is USART2,
 * because its package bonds neither of USART1's pin pairs.
 *
 * THE FOURTH PORT IS A USART ON THIS CLASS. Chapter 18's opening counts
 * three USARTs and five UARTs for the whole family and then names the
 * exception: on the CH32V203C8 the fourth serial port is a USART4 - the
 * datasheet's pin table agrees, giving it a CK, a CTS and an RTS pad -
 * while on the CH32V203RB (the other device class) it is a UART4 with
 * TX and RX alone. So `is_full` - the synchronous clock, the smartcard
 * and the flow-control pair - is a PART fact here (device::usart_full)
 * and not a number's parity, and the verbs behind it refuse rather than
 * write bits an instance has not got.
 *
 * THE PADS ARE THE REMAP TABLES' (afio.hpp, tables 10-23..10-27): the
 * `remap` template parameter names a COLUMN and the TX, RX, CK, CTS and
 * RTS pads follow from the same table that programs AFIO_PCFR1/PCFR2.
 * A code of 0 writes nothing at all - the reset column is already
 * there, and USART1 carries the probe's console on this board, which is
 * a pair of pads no init() may move behind the program's back. UART4 is
 * where the table matters most: the manual has two of them and the one
 * that starts at PC10/PC11 belongs to the bigger class, the CH32V203C8
 * being named in the other, whose default pads are PB0 and PB1.
 *
 * TWO RINGS AND TWO FLAGS. Bytes leave through a TX ring drained by the
 * TXE interrupt (write_byte() arms TXEIE; the ISR disarms it when the
 * ring runs dry, because TXE stands for ever while the transmitter is
 * idle and would re-enter the handler without end), and arrive into an
 * RX ring filled by RXNE. isr() returns the "RX went non-empty" EDGE
 * that util/serial_port.hpp posts on - one event per idle-to-busy
 * transition, not one per byte.
 *
 * THE BAUD DIVISOR IS THE WHOLE REGISTER. BRR counts the peripheral
 * clock's periods per bit in sixteenths (18.3: a 12-bit mantissa and a
 * 4-bit fraction), so the value to store is simply pclk / baud - no
 * field arithmetic - and actual_baud() inverts it, which is how a
 * program reports the rate it is really running at. Below 16 the
 * generator has nothing to divide by: refused.
 *
 * ERRORS ARE READ THEN CLEARED. Parity, framing, noise, overrun and the
 * idle line stand in STATR and go away when STATR is read and then
 * DATAR, in that order (18.10.1); RXNE, TC, LBD and CTS also clear by
 * writing a zero over them. The handler therefore reads the status
 * ONCE, decides from that copy, and lets the DATAR read do the
 * clearing. A byte that arrived with an error is DROPPED rather than
 * pushed - a corrupted byte in a line assembler is worse than a gap -
 * and the counter says it happened.
 *
 * THE MODES EXCLUDE EACH OTHER THE WAY 18.4 .. 18.7 SAY, and the
 * exclusions are NOT symmetrical: the synchronous clock wants SCEN,
 * HDSEL and IREN clear; half duplex wants SCEN, CLKEN and IREN clear;
 * the smartcard wants LINEN, HDSEL and IREN clear and KEEPS CLKEN,
 * which is where its card clock comes from - so smartcard() writes that
 * bit itself, there being no other verb the exclusions would let reach
 * it under SCEN; IrDA wants LINEN, STOP, CLKEN, SCEN and HDSEL clear.
 * Each verb refuses instead of storing a combination the chapter
 * declares undefined.
 *
 * TWO OPTIONAL DMA ENGINE SLOTS, the shape every stratum with a
 * controller uses: a transmit engine drains the TX ring by contiguous
 * runs (the ring's read_span, consumed by exactly what the block
 * carried) and a receive engine fills the RX ring's free run
 * (write_span, published by what harvest() finds arrived). Without an
 * engine the slot is the NoDmaEngine tag, every engine branch is
 * compiled out and init() does not so much as touch CTLR3. With a
 * receive engine RXNE belongs to the channel, so the error flags are
 * read once per harvest and counted against the RUN and not the byte -
 * a console that wants exact attribution takes no receive engine. And
 * harvest() is a VERB, not an interrupt: a receive block completes only
 * when its run fills, which on an idle line is never, so whoever owns
 * the port decides how often to ask. An engine is refused on any
 * channel but the instance's own, which the request table of
 * ch32v203/dma_engine.hpp answers (USART1 transmits on channel 4 and
 * receives on 5, USART2 on 7 and 6, USART3 on 2 and 3, UART4 on 1
 * and 8) - and a program with an engine running does not sleep on this
 * family, because in Sleep the bus matrix serves the core alone
 * (docs/ch32v203/dma.md).
 *
 * WHAT THE REGISTER FILE HAS NOT GOT. CTLR4 - the MARK and SPACE parity
 * of the bigger classes - is not this family's (18.10.8's note), and
 * neither are CTLR1's M_EXT (five, six and seven-bit words) nor STATR's
 * MS_ERR and RX_BUSY: every one of them carries the same note naming
 * the CH32F20x_D8, the CH32V30x and the CH32V31x. device.hpp's register
 * view stops at GPR for that reason, and nothing here reaches past it.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/afio.hpp"
#include "ch32v203/clock.hpp"
#include "ch32v203/device.hpp"
#include "ch32v203/dma_engine.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

// =============================================================================
// The chapter's vocabulary
// =============================================================================

/// DATA bits of a frame. The register speaks in WORD length (M: 8 or 9
/// bits including the parity bit when there is one), so seven data bits
/// exist only with a parity bit and nine only without.
enum class UartBits : uint8_t { seven = 7, eight = 8, nine = 9 };

enum class UartParity : uint8_t { none, even, odd };

/// CTLR2.STOP, all four codes; the half and one-and-a-half stops are the
/// chapter's own (18.10.5).
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

/// CTLR1's M, PCE and PS for a format (the word length is the data bits
/// plus the parity bit).
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

/// Mute mode (18.10.4's RWU and WAKE, 18.10.5's ADD): the receiver
/// asleep until the line goes idle, or until a frame whose MSB is set
/// carries this node's 4-bit address.
enum class MuteWake : uint8_t { idle_line, address_mark };

struct MuteConfig {
    MuteWake wake = MuteWake::idle_line;
    uint8_t address = 0;   ///< 0..15, for address_mark
};

constexpr bool mute_valid(const MuteConfig& c) { return c.address <= 15u; }

/// LIN mode (18.10.5's LINEN): the break SBK sends, and its detection at
/// ten or eleven bits with its own flag and interrupt.
struct LinConfig {
    bool break_11bit = false;   ///< LBDL: 11-bit detection instead of 10
    bool break_interrupt = false;
};

/// IrDA (18.7, 18.10.7): the SIR encoder on TX and decoder on RX, normal
/// mode at the bit rate (a 3/16 pulse) or low-power mode on the
/// prescaled clock. GPR.PSC is all eight bits in low-power mode and
/// must be 1 in normal mode; a prescaler of 0 "means reservation" and is
/// refused.
struct IrdaConfig {
    bool low_power = false;
    uint8_t prescaler = 1;   ///< GPR.PSC
};

constexpr bool irda_valid(const IrdaConfig& c) {
    return c.prescaler != 0u && (c.low_power || c.prescaler == 1u);
}

/// The synchronous mode's three choices (18.4, figure 18-2): the clock's
/// idle level (CPOL), the capture edge (CPHA) and whether the last data
/// bit gets a clock pulse too (LBCL). THE SENSE OF LBCL IS THIS
/// MANUAL'S, and it is the opposite of the F1 family's: 18.10.5 reads
/// "0: the clock pulse of the last bit of data is output from CK; 1: ...
/// is not output". `last_bit_clock` names the BIT and not a promise -
/// what the pad does is docs/ch32v203/usart.md's measurement.
struct UsartSyncConfig {
    bool clock_idle_high = false;      ///< CPOL
    bool capture_second_edge = false;  ///< CPHA
    bool last_bit_clock = false;       ///< LBCL, as the register spells it
};

/// The smartcard (18.6): ISO 7816-3 on a single wire with the card's
/// clock on CK - a NACK on a parity error, the guard time in bit times
/// (GPR.GT) and the clock prescaler (GPR.PSC, the low FIVE bits, the
/// source divided by twice the value, so 31 is the deepest division and
/// 0 is reserved).
struct SmartcardConfig {
    bool nack = true;
    uint8_t guard_time = 0;
    uint8_t clock_prescaler = 1;
};

constexpr bool smartcard_valid(const SmartcardConfig& c) {
    return c.clock_prescaler != 0u && c.clock_prescaler <= 31u;
}

/// The divisor this clock and baud ask for: pclk/baud in sixteenths,
/// rounded to nearest so the error is halved. Below 16 there is no whole
/// clock period per sixteenth of a bit and the generator has nothing to
/// divide by - init() refuses such a value.
constexpr uint32_t usart_divisor(uint32_t pclk, uint32_t baud) {
    return baud == 0u ? 0u : (pclk + baud / 2u) / baud;
}

constexpr bool usart_divisor_valid(uint32_t brr) { return brr >= 16u && brr <= 0xFFFFu; }

/// The rate a divisor really gives at this clock, and the smallest clock
/// that can produce a rate at all (sixteen periods a bit).
constexpr uint32_t usart_actual_baud(uint32_t pclk, uint32_t brr) {
    return brr == 0u ? 0u : pclk / brr;
}
constexpr uint32_t usart_min_hz(uint32_t baud) { return baud * 16u; }

/// Which bus an instance answers on, and therefore which gate opens it
/// and which clock its divisor counts.
constexpr Bus usart_bus_for(uint8_t n) { return n == 1 ? Bus::pb2 : Bus::pb1; }

constexpr uint32_t usart_gate_for(uint8_t n) {
    return n == 1 ? rcc_pb2_usart1 :
           n == 2 ? rcc_pb1_usart2 :
           n == 3 ? rcc_pb1_usart3 :
           n == 4 ? rcc_pb1_uart4 : 0;
}

constexpr Irq usart_irq_for(uint8_t n) {
    return n == 1 ? Irq::usart1 :
           n == 2 ? Irq::usart2 :
           n == 3 ? Irq::usart3 : Irq::uart4;
}

/// The AFIO field an instance's column is selected by.
constexpr Remap usart_remap_of(uint8_t n) {
    return n == 1 ? Remap::usart1 :
           n == 2 ? Remap::usart2 :
           n == 3 ? Remap::usart3 : Remap::uart4;
}

/// Whether a column exists for this instance ON THIS PART - the device
/// class has it and the package bonds at least one of its pads, which is
/// afio.hpp's one judgement and not a second table here.
constexpr bool usart_remap_valid(uint8_t n, uint8_t code) {
    return afio_remap_has_code(usart_remap_of(n), code);
}

/// The two pads a transport claims, out of the five a column carries.
struct UsartPads {
    Pad tx;
    Pad rx;
};

/// The whole column - TX, RX, CK, CTS, RTS - under a remap code. ONE
/// table answers, afio.hpp's, which is also what writes the register.
constexpr UsartPadSet usart_column_for(uint8_t n, uint8_t code = 0) {
    return afio_usart_pads(n, code);
}

constexpr UsartPads usart_pads_for(uint8_t n, uint8_t code = 0) {
    const UsartPadSet p = usart_column_for(n, code);
    return UsartPads{p.tx, p.rx};
}

/// The DMA channel each direction of an instance answers on, read out
/// of the one request table (ch32v203/dma_engine.hpp): 0 where the part
/// has not got the instance at all. THE CHANNEL IS THE REQUEST here, so
/// these two numbers are what an engine slot is checked against.
constexpr uint8_t usart_dma_tx_channel(uint8_t n) {
    return n == 1 ? dma_request_channel(DmaRequest::usart1_tx) :
           n == 2 ? dma_request_channel(DmaRequest::usart2_tx) :
           n == 3 ? dma_request_channel(DmaRequest::usart3_tx) :
           n == 4 ? dma_request_channel(DmaRequest::uart4_tx) : 0;
}

constexpr uint8_t usart_dma_rx_channel(uint8_t n) {
    return n == 1 ? dma_request_channel(DmaRequest::usart1_rx) :
           n == 2 ? dma_request_channel(DmaRequest::usart2_rx) :
           n == 3 ? dma_request_channel(DmaRequest::usart3_rx) :
           n == 4 ? dma_request_channel(DmaRequest::uart4_rx) : 0;
}

// =============================================================================
// Usart<n>: the resource
// =============================================================================

/**
 * One USART instance, register by register. Every verb stores what it
 * names and nothing else; a verb whose combination the chapter declares
 * undefined refuses (false, nothing written). The task below is written
 * on these verbs; a program that wants the chapter beyond the transport
 * - a break on a LIN line, a muted receiver, a clocked frame - reaches
 * them here.
 */
template <uint8_t n>
struct Usart {
    static_assert(usart_base_for(n) != 0, "brio Usart: this family has USART1..3 and UART4");
    static_assert(device::has_usart(n),
                  "brio Usart: this part does not offer that instance (parts/<part>.hpp)");

    Usart() = delete;

    static constexpr uint8_t number = n;
    static constexpr Bus bus = usart_bus_for(n);
    static constexpr UsartPads pads = usart_pads_for(n);
    static constexpr Irq irq = usart_irq_for(n);
    static constexpr uint8_t dma_tx_channel = usart_dma_tx_channel(n);
    static constexpr uint8_t dma_rx_channel = usart_dma_rx_channel(n);

    /// Whether this instance is a full USART - the synchronous clock,
    /// the smartcard and the flow-control pair - or an asynchronous
    /// receiver alone. A PART fact (the file header): the fourth port is
    /// a USART4 on the CH32V203C8 and a UART4 on the other class.
    static constexpr bool is_full = device::usart_full(n);

    static UsartRegs& regs() { return *reinterpret_cast<UsartRegs*>(usart_base_for(n)); }

    /// The peripheral's clock gate. Nothing in this file reads a
    /// register before this is on: an unclocked block answers rubbish.
    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(bus, usart_gate_for(n));
        } else {
            Rcc::disable(bus, usart_gate_for(n));
        }
    }

    /// Put the block back to its reset state, gate left as it is.
    static void reset() { Rcc::reset(bus, usart_gate_for(n)); }

    /// Select the instance's column (afio.hpp). False - and nothing
    /// written - for a column this part has not got. Code 0 is the reset
    /// column and writing it is still a write, which is why the
    /// transport skips the call entirely at that code.
    static bool remap(uint8_t code) {
        Afio::clock_on();
        return Afio::remap(usart_remap_of(n), code);
    }

    // ---- configuration ----------------------------------------------------

    /// The frame and the divisor, with the port DISABLED: CTLR1's frame
    /// bits and CTLR2's stop bits may only be trusted while UE is clear
    /// or the transmitter idle, and a divisor written under traffic
    /// lands mid-frame. The two registers are written WHOLE - this is
    /// the from-scratch verb, and the modes below are what a program
    /// adds to it afterwards.
    static bool configure(const UartFormat& f, uint32_t brr) {
        if (!uart_format_valid(f) || !usart_divisor_valid(brr)) {
            return false;
        }
        regs().CTLR1 = usart_ctlr1_format(f);
        regs().CTLR2 = usart_ctlr2_stop(f.stop);
        regs().BRR = static_cast<uint16_t>(brr);
        return true;
    }

    /// The stop bits alone. Refused under IrDA, which 18.7 wants at one.
    static bool stop_bits(UartStop s) {
        if ((regs().CTLR3 & usart_iren) != 0u && s != UartStop::one) {
            return false;
        }
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_stop_mask) | usart_ctlr2_stop(s));
        return true;
    }

    static bool set_brr(uint32_t v) {
        if (!usart_divisor_valid(v)) {
            return false;
        }
        regs().BRR = static_cast<uint16_t>(v);
        return true;
    }
    static uint16_t brr() { return regs().BRR; }

    static bool enabled() { return (regs().CTLR1 & usart_ue) != 0u; }

    static void enable(bool on) {
        if (on) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_ue);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_ue);
        }
    }

    static void transmitter(bool on) {
        regs().CTLR1 = static_cast<uint16_t>(on ? (regs().CTLR1 | usart_te)
                                               : (regs().CTLR1 & ~usart_te));
    }

    static void receiver(bool on) {
        regs().CTLR1 = static_cast<uint16_t>(on ? (regs().CTLR1 | usart_re)
                                               : (regs().CTLR1 & ~usart_re));
    }

    // ---- the receiver's modes ---------------------------------------------

    /// Mute mode: WAKE and ADD written, RWU left to mute()/unmute()
    /// because 18.10.4's note 1 says the receiver must have taken a byte
    /// before an idle-line wake can work.
    static bool mute_mode(const MuteConfig& c) {
        if (!mute_valid(c)) {
            return false;
        }
        bit(regs().CTLR1, usart_wake, c.wake == MuteWake::address_mark);
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_add_mask) | c.address);
        return true;
    }
    /// Put the receiver to sleep. 18.10.4's note 2: under an address-mark
    /// wake RWU cannot be written while RXNE stands - refused then.
    static bool mute() {
        if ((regs().CTLR1 & usart_wake) != 0u && (regs().STATR & usart_rxne) != 0u) {
            return false;
        }
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_rwu);
        return true;
    }
    static void unmute() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_rwu); }
    static bool muted() { return (regs().CTLR1 & usart_rwu) != 0u; }

    // ---- the line's modes -------------------------------------------------

    /// LIN mode. Refused while half duplex, IrDA, the smartcard or the
    /// clock is on - a break is a line's whole frame time and none of
    /// those shapes has room for it.
    static bool lin(const LinConfig& c) {
        if ((regs().CTLR2 & usart_clken) != 0u ||
            (regs().CTLR3 & (usart_hdsel | usart_iren | usart_scen)) != 0u) {
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

    /// SBK: one break frame after the current one; the bit clears itself
    /// when the break's stop bit is out (18.10.4). Ten or eleven bits of
    /// low outside LIN mode, thirteen inside it.
    static void send_break() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_sbk); }
    static bool break_pending() { return (regs().CTLR1 & usart_sbk) != 0u; }

    /// Single-wire half duplex (18.5): the TX pad alone carries both
    /// directions and the chapter asks for it as an OPEN-DRAIN output,
    /// because the bus has more than one talker on it. Refused while
    /// LIN, IrDA, the smartcard or the clock is on.
    static bool half_duplex(bool on) {
        if (on && ((regs().CTLR2 & (usart_linen | usart_clken)) != 0u ||
                   (regs().CTLR3 & (usart_iren | usart_scen)) != 0u)) {
            return false;
        }
        bit(regs().CTLR3, usart_hdsel, on);
        return true;
    }
    static bool half_duplex() { return (regs().CTLR3 & usart_hdsel) != 0u; }

    /// IrDA (18.7): refused while LIN, half duplex, the smartcard or the
    /// clock is on, with stop bits other than one, and with a prescaler
    /// the register description calls reserved.
    static bool irda(const IrdaConfig& c) {
        if (!irda_valid(c)) {
            return false;
        }
        if ((regs().CTLR2 & (usart_linen | usart_clken)) != 0u ||
            (regs().CTLR3 & (usart_hdsel | usart_scen)) != 0u) {
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

    /// GPR's two fields as they stand: the prescaler both IrDA and the
    /// smartcard use, and the guard time the smartcard alone does.
    static uint8_t prescaler() { return static_cast<uint8_t>(regs().GPR & usart_psc_mask); }
    static uint8_t guard_time() { return static_cast<uint8_t>((regs().GPR & usart_gt_mask) >> 8); }

    /// The smartcard (18.6), a FULL instance's. Sets the guard time and
    /// the card-clock prescaler, 1.5 stop bits, CLKEN and SCEN: the
    /// clock is the CARD'S and belongs to the mode, which is why this
    /// verb writes it rather than leaving it to a synchronous() the
    /// chapter's own exclusions would refuse under SCEN. What is not
    /// written is the CK pad - the caller hands that over
    /// (usart_column_for(n, code).ck) - and CPOL, CPHA and LBCL, which
    /// keep whatever a program chose. Refused while LIN, half duplex or
    /// IrDA is on.
    static bool smartcard(const SmartcardConfig& c) {
        if constexpr (!is_full) {
            (void)c;
            return false;
        } else {
            if (!smartcard_valid(c)) {
                return false;
            }
            if ((regs().CTLR2 & usart_linen) != 0u ||
                (regs().CTLR3 & (usart_hdsel | usart_iren)) != 0u) {
                return false;
            }
            regs().GPR = static_cast<uint16_t>((static_cast<uint16_t>(c.guard_time) << 8) |
                                               c.clock_prescaler);
            regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_stop_mask) |
                                                 usart_ctlr2_stop(UartStop::one_and_half) |
                                                 usart_clken);
            uint16_t v = static_cast<uint16_t>(regs().CTLR3 & ~usart_nack);
            if (c.nack) { v |= usart_nack; }
            v |= usart_scen;
            regs().CTLR3 = v;
            return true;
        }
    }
    /// SCEN and its NACK dropped. The CARD CLOCK IS LEFT RUNNING: CLKEN
    /// is the synchronous half's bit and synchronous_off() is what stops
    /// it, under its own TE/RE rule.
    static void smartcard_off() {
        if constexpr (is_full) {
            regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 & ~(usart_scen | usart_nack));
        }
    }
    static bool smartcard_enabled() {
        if constexpr (!is_full) {
            return false;
        } else {
            return (regs().CTLR3 & usart_scen) != 0u;
        }
    }

    /// THE SYNCHRONOUS MODE (18.4), a FULL instance's: the column's CK
    /// pad carries a clock while the TRANSMITTER shifts and at no other
    /// time, and the receiver samples on it - this side is the master
    /// and CK is an output only. Refused while LIN, half duplex, IrDA or
    /// the smartcard is on, and while the transmitter or the receiver is
    /// enabled, because CPOL, CPHA and LBCL "need to be set when TE and
    /// RE are not enabled". The CK pad itself is the caller's to hand to
    /// the peripheral (usart_column_for(n, code).ck).
    static bool synchronous(const UsartSyncConfig& c) {
        if constexpr (!is_full) {
            (void)c;
            return false;
        } else {
            if ((regs().CTLR2 & usart_linen) != 0u ||
                (regs().CTLR3 & (usart_hdsel | usart_iren | usart_scen)) != 0u) {
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
        if constexpr (!is_full) {
            return false;
        } else {
            if ((regs().CTLR1 & (usart_te | usart_re)) != 0u) {
                return false;
            }
            regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 &
                                                 ~(usart_clken | usart_cpol | usart_cpha | usart_lbcl));
            return true;
        }
    }
    static bool synchronous_enabled() {
        if constexpr (!is_full) {
            return false;
        } else {
            return (regs().CTLR2 & usart_clken) != 0u;
        }
    }

    /// The hardware flow-control pair (18.10.6), a FULL instance's: RTS
    /// driven low while the receiver can take a frame, CTS sampled
    /// before each frame goes out.
    static bool flow_control(bool rts, bool cts) {
        if constexpr (!is_full) {
            (void)rts;
            (void)cts;
            return false;
        } else {
            bit(regs().CTLR3, usart_rtse, rts);
            bit(regs().CTLR3, usart_ctse, cts);
            return true;
        }
    }
    static bool rts_enabled() { return (regs().CTLR3 & usart_rtse) != 0u; }
    static bool cts_enabled() { return (regs().CTLR3 & usart_ctse) != 0u; }

    /// CTLR3's two DMA request enables (18.10.6). With DMAT set, TXE
    /// raises a request instead of feeding the interrupt; with DMAR
    /// set, so does RXNE - which is why a receiver with an engine
    /// leaves RXNEIE alone.
    static void dma_transmit(bool on) {
        regs().CTLR3 = static_cast<uint16_t>(on ? (regs().CTLR3 | usart_dmat)
                                               : (regs().CTLR3 & ~usart_dmat));
    }
    static void dma_receive(bool on) {
        regs().CTLR3 = static_cast<uint16_t>(on ? (regs().CTLR3 | usart_dmar)
                                               : (regs().CTLR3 & ~usart_dmar));
    }

    /// Where an engine points: the one register both directions share.
    static volatile void* data_address() { return static_cast<volatile void*>(&regs().DATAR); }

    // ---- interrupts and flags ---------------------------------------------

    /// CTLR1's five enables by mask: usart_peie, usart_txeie, usart_tcie,
    /// usart_rxneie (ORE rides it), usart_idleie.
    static void interrupts(uint16_t ctlr1_mask, bool on) { bit(regs().CTLR1, ctlr1_mask, on); }
    static void rxne_interrupt(bool on) { bit(regs().CTLR1, usart_rxneie, on); }
    static void txe_interrupt(bool on) { bit(regs().CTLR1, usart_txeie, on); }
    static bool txe_interrupt() { return (regs().CTLR1 & usart_txeie) != 0u; }
    static void tc_interrupt(bool on) { bit(regs().CTLR1, usart_tcie, on); }
    static void idle_interrupt(bool on) { bit(regs().CTLR1, usart_idleie, on); }
    static void parity_interrupt(bool on) { bit(regs().CTLR1, usart_peie, on); }
    static void break_interrupt(bool on) { bit(regs().CTLR2, usart_lbdie, on); }
    static void cts_interrupt(bool on) { bit(regs().CTLR3, usart_ctsie, on); }
    /// EIE: FE, ORE and NE raise the vector - under DMAR only (18.10.6).
    static void error_interrupt(bool on) { bit(regs().CTLR3, usart_eie, on); }

    static uint16_t status() { return regs().STATR; }
    static bool flag(uint16_t mask) { return (regs().STATR & mask) != 0u; }
    /// The write-zero-to-clear flags (RXNE, TC, LBD, CTS); any other bit
    /// in the mask is ignored, because writing it does nothing.
    static void clear_flags(uint16_t mask) {
        regs().STATR = static_cast<uint16_t>(~(mask & usart_statr_rw0));
    }
    /// The read-sequence clear of IDLE, ORE, NE, FE and PE (and of RXNE,
    /// whose byte this discards).
    static void clear_by_read() {
        (void)regs().STATR;
        (void)regs().DATAR;
    }

    // ---- the line ---------------------------------------------------------

    static bool tx_empty() { return (regs().STATR & usart_txe) != 0u; }
    static bool tx_complete() { return (regs().STATR & usart_tc) != 0u; }
    static bool rx_ready() { return (regs().STATR & usart_rxne) != 0u; }

    /// The word as the receiver has it - nine bits when the frame is
    /// nine bits wide. Reading DATAR is what clears RXNE, and the
    /// STATR-then-DATAR pair is what clears the error flags.
    static uint16_t read_word() { return static_cast<uint16_t>(regs().DATAR); }
    static void write_word(uint16_t w) { regs().DATAR = w; }
    static uint8_t read_data() { return static_cast<uint8_t>(regs().DATAR & 0xFFu); }
    static void write_data(uint8_t b) { regs().DATAR = b; }

    /// Read the errors and clear them, the sequence the chapter
    /// prescribes (18.10.1): the status register, then the data one.
    static uint16_t take_errors() {
        const uint16_t status = regs().STATR;
        const uint16_t errors = static_cast<uint16_t>(status & (usart_pe | usart_fe | usart_ne | usart_ore));
        if (errors != 0u) {
            (void)regs().DATAR;
        }
        return errors;
    }

    /// The baud the port is ACTUALLY running at, from the divisor in the
    /// register - what a program reports instead of what it asked for.
    static uint32_t actual_baud(uint32_t pclk) {
        const uint32_t brr = regs().BRR;
        return brr == 0u ? 0u : pclk / brr;
    }

private:
    static void bit(volatile uint16_t& r, uint16_t mask, bool on) {
        r = static_cast<uint16_t>(on ? (r | mask) : (r & ~mask));
    }
};

// =============================================================================
// Uart: the transport task
// =============================================================================

/// The peripheral clock an instance counts its divisor in: USART1 is the
/// PB2 one and the other three are PB1's, and the two buses do NOT run
/// at the same rate above the cap ch32v203/clock.hpp states.
template <uint8_t instance, typename C>
constexpr uint32_t usart_bus_hz(C clock) {
    if constexpr (C::is_static) {
        (void)clock;
        return instance == 1u ? C::pclk2_hz : C::pclk1_hz;
    } else {
        (void)clock;
        return instance == 1u ? C::pclk2_hz() : C::pclk1_hz();
    }
}

/// The same answer from an HCLK, which is what a dynamic clock hands a
/// user it is about to rebase: the RCC still holds the prescalers of the
/// rate being left, so the new bus rate is arithmetic and not a read.
template <uint8_t instance>
constexpr uint32_t usart_bus_hz_at(uint32_t hclk) {
    return instance == 1u ? pclk2_hz_at(hclk) : pclk1_hz_at(hclk);
}

/**
 * What a Uart may be told beyond its frame, its rings and its engines,
 * as one trailing template parameter. The FRAME IS NOT IN HERE on this
 * family - it has a parameter of its own, ahead of the engine slots -
 * which is this stratum's spelling of the CH32V00x's UartOptions
 * (docs/design/serial.md's realizations table records it). The defaults
 * are the console's personality and compile to exactly the code they
 * did before this struct existed.
 */
struct UartOptions {
    /// Single-wire half duplex (18.5): one pad, the TX pad, as an
    /// alternate-function OPEN DRAIN against an external pull-up; the RX
    /// pad is left alone. What the instance's own receiver hears of what
    /// it sends is docs/ch32v203/usart.md's measurement.
    bool half_duplex = false;
    /// The flow-control pair on the column's CTS and RTS pads, on a full
    /// instance (Usart<n>::is_full).
    bool rts = false;
    bool cts = false;
};

/**
 * The interrupt-driven serial port.
 *
 *   using Serial = brio::Uart<1, Platform>;
 *   constexpr Serial serial;                 // tag for print(serial, ...)
 *   Serial::init(clock, 115200);
 *   extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { Serial::isr(); }
 *
 * `P` is the platform, which the rings need to know whether an index can
 * be shared with a handler bare (atomic_width) or wants a guard.
 */
template <uint8_t instance, typename P, uint16_t rx_size = 64, uint16_t tx_size = 64,
          UartFormat format = {}, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine, uint8_t remap = 0, UartOptions opts = {}>
struct Uart {
    static_assert(usart_base_for(instance) != 0,
                  "brio Uart: this family has USART1..3 and UART4");
    static_assert(device::has_usart(instance),
                  "brio Uart: this part does not offer that instance (parts/<part>.hpp)");
    static_assert(uart_format_valid(format) && format.bits != UartBits::nine,
                  "brio Uart: the rings carry bytes - seven data bits with a parity bit, or eight, "
                  "with or without one; nine-bit words are the resource's read_word()/write_word()");
    static_assert(usart_remap_valid(instance, remap),
                  "brio Uart: no such column for this instance on this part (afio.hpp's tables "
                  "10-23 to 10-27 - the device class has it and the package bonds its pads)");
    static_assert(!opts.rts || device::usart_full(instance),
                  "brio Uart: the flow-control pair is a full USART's - the fourth port has it on "
                  "the CH32V203C8 and not on the other class (18.10.6)");
    static_assert(!opts.cts || device::usart_full(instance),
                  "brio Uart: the flow-control pair is a full USART's - the fourth port has it on "
                  "the CH32V203C8 and not on the other class (18.10.6)");
    static_assert(!opts.rts || pad_bonded(usart_column_for(instance, remap).rts),
                  "brio Uart: this package does not bond this column's RTS pad - the signal exists "
                  "and the pin does not (parts/<part>.hpp)");
    static_assert(!opts.cts || pad_bonded(usart_column_for(instance, remap).cts),
                  "brio Uart: this package does not bond this column's CTS pad - the signal exists "
                  "and the pin does not (parts/<part>.hpp)");
    // An engine is checked where it is NAMED: sizeof demands a complete
    // type, so an engine's own static_asserts fire on the line the
    // application wrote.
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / DmaRxEngine from "
                  "ch32v203/dma.hpp, or NoDmaEngine (the default)");
    static_assert(!TxEngine::present ||
                      dma_engine_channel<TxEngine>() == usart_dma_tx_channel(instance),
                  "brio Uart: table 11-5 - this instance transmits on its own DMA channel "
                  "(USART1 on 4, USART2 on 7, USART3 on 2, UART4 on 1)");
    static_assert(!RxEngine::present ||
                      dma_engine_channel<RxEngine>() == usart_dma_rx_channel(instance),
                  "brio Uart: table 11-5 - this instance receives on its own DMA channel "
                  "(USART1 on 5, USART2 on 6, USART3 on 3, UART4 on 8)");
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

    using Tx = Pin<pads.tx.port, pads.tx.pin>;
    using Rx = Pin<pads.rx.port, pads.rx.pin>;

    static UsartRegs& regs() { return Resource::regs(); }

    /**
     * Open the port at `baud` in the given frame (8N1 by default), with
     * the receive interrupt armed.
     *
     * Returns false when the divisor comes out below the smallest the
     * chapter allows (16, i.e. one whole peripheral-clock period per bit
     * sixteenth): a baud rate this clock cannot serve is a fact the
     * caller must see rather than a port that mangles every frame.
     */
    template <typename C>
    static bool init(C clock, uint32_t baud) {
        static_assert(clock_follows<C, Uart>(),
                      "brio Uart: the baud divisor is derived from the peripheral clock, so a "
                      "dynamic clock must list this port among the users it rebases");

        const uint32_t pclk = usart_bus_hz<instance>(clock);
        const uint32_t brr = usart_divisor(pclk, baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }

        Resource::bus_clock(true);

        // The column, only where it is not the reset one: AFIO's reset
        // value already selects code 0, and USART1 carries this board's
        // console - a write there would move the console's pads.
        if constexpr (remap != 0u) {
            (void)Resource::remap(remap);
        }

        // Pads before the enable: TE's idle frame must land on a pad the
        // peripheral already owns.
        if constexpr (opts.half_duplex) {
            Tx::function(PinDrive::open_drain);   // the one wire; RX is left alone
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

        if (!Resource::configure(format, brr)) {
            return false;
        }
        m_baud = baud;

        m_tx.clear();
        m_rx.clear();
        m_rx_overruns = 0;
        m_hw_overruns = 0;
        m_frame_errors = 0;
        m_noise_errors = 0;
        m_parity_errors = 0;

        // CTLR3 IS TOUCHED ONLY BY A PORT THAT HAS SOMETHING TO PUT IN
        // IT: with no engine and no option there is nothing of this
        // transport's there, and whatever a program stored itself stays.
        if constexpr (has_tx_engine || has_rx_engine || opts.half_duplex || opts.rts || opts.cts) {
            m_dma_faults = 0;
            uint16_t ctlr3 = regs().CTLR3;
            if constexpr (has_tx_engine) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_dmat); }
            if constexpr (has_rx_engine) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_dmar); }
            if constexpr (opts.half_duplex) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_hdsel); }
            if constexpr (opts.rts) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_rtse); }
            if constexpr (opts.cts) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_ctse); }
            regs().CTLR3 = ctlr3;
        }

        // RXNE belongs to the receive channel when an engine has it, so
        // the per-byte interrupt is armed only where no engine is.
        regs().CTLR1 = static_cast<uint16_t>(usart_ctlr1_format(format) | usart_ue | usart_te |
                                             usart_re |
                                             (has_rx_engine ? uint16_t{0} : usart_rxneie));

        if constexpr (has_rx_engine) {
            RxEngine::arm(Resource::data_address());
            rearm_rx();
        }
        if constexpr (has_tx_engine) {
            TxEngine::arm(Resource::data_address());
        }

        Pfic::enable(usart_irq_for(instance));
        return true;
    }

    /**
     * The ISR body of WHICHEVER DMA channel this transport owns - bind
     * it to the vector(s) the engines' channels report on:
     *
     *     extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Serial::dma_isr(); }
     *
     * Each engine reads only its own channel's flags, so this is safe
     * on a vector another channel of the program shares nothing with.
     * On the transmit channel a completion releases exactly the block's
     * bytes from the ring and starts the next run; on the receive
     * channel nothing is published here - harvest() does that.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        bool mine = false;
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service();
            if ((f & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                bump(m_dma_faults);
                mine = true;
            } else if ((f & TxEngine::flag_complete) != 0u) {
                m_tx.consume(static_cast<typename decltype(m_tx)::index_t>(TxEngine::complete()));
                pump_tx();
                mine = true;
            }
        }
        if constexpr (has_rx_engine) {
            const uint8_t f = RxEngine::service();
            if ((f & RxEngine::flag_error) != 0u) {
                (void)RxEngine::abandon();
                bump(m_dma_faults);
                mine = true;
            } else if ((f & RxEngine::flag_complete) != 0u) {
                mine = true;   // the run filled: harvest() publishes and re-arms
            }
        }
        return mine;
    }

    /**
     * Ask the receive engine what has arrived, and publish it. A VERB,
     * not an interrupt (see the file header): the owner decides how
     * often, a kernel TimeEvent every few ticks is the shape. Returns
     * the same edge isr() does - the ring went from empty to non-empty.
     * False, and free, without an engine.
     */
    static bool harvest() {
        if constexpr (!has_rx_engine) {
            return false;
        } else {
            // The error flags once, at harvest granularity, cleared by
            // the STATR-then-DATAR read the chapter prescribes.
            const uint16_t status = regs().STATR;
            if ((status & (usart_fe | usart_ne | usart_pe | usart_ore)) != 0u) {
                if ((status & usart_fe) != 0u) { bump(m_frame_errors); }
                if ((status & usart_ne) != 0u) { bump(m_noise_errors); }
                if ((status & usart_pe) != 0u) { bump(m_parity_errors); }
                if ((status & usart_ore) != 0u) { bump(m_hw_overruns); }
                (void)regs().DATAR;
            }
            const bool was_empty = m_rx.empty();
            const uint16_t fresh = RxEngine::take();
            if (fresh != 0u) {
                m_rx.publish(static_cast<typename decltype(m_rx)::index_t>(fresh));
            }
            // The silicon is asked first and the arithmetic second: a
            // channel that is not running gets a new run whatever the
            // count says.
            if (RxEngine::idle() || RxEngine::full() || RxEngine::capacity() == 0u) {
                rearm_rx();
            }
            return was_empty && !m_rx.empty();
        }
    }

    /// Blocks the engines threw away. Always 0, and free, without one.
    static uint16_t dma_faults() { return m_dma_faults; }

    /**
     * The port's whole interrupt body - bind this instance's vector to
     * it.
     *
     * Returns true when a byte arrived into an EMPTY ring: the edge
     * util/serial_port.hpp posts on.
     */
    // always_inline: one call site (the app's vector binding), so the
    // compiler saves only the registers this body uses.
    [[gnu::always_inline]] static bool isr() {
        bool rx_edge = false;

        const uint16_t status = regs().STATR;

        if ((status & usart_rxne) != 0u) {
            const uint8_t byte = static_cast<uint8_t>(regs().DATAR & 0xFFu);
            if ((status & (usart_fe | usart_ne | usart_pe | usart_ore)) != 0u) {
                if ((status & usart_fe) != 0u) { bump(m_frame_errors); }
                if ((status & usart_ne) != 0u) { bump(m_noise_errors); }
                if ((status & usart_pe) != 0u) { bump(m_parity_errors); }
                if ((status & usart_ore) != 0u) { bump(m_hw_overruns); }
            } else {
                const bool was_empty = m_rx.empty();
                if (m_rx.push(byte)) {
                    rx_edge = was_empty;
                } else {
                    bump(m_rx_overruns);
                }
            }
        }

        if ((status & usart_txe) != 0u && (regs().CTLR1 & usart_txeie) != 0u) {
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
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            return false;
        }
        if constexpr (has_tx_engine) {
            pump_tx();
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_txeie);
        }
        return true;
    }

    /// Take one received byte; false when none is pending.
    static bool read_byte(uint8_t& b) {
        const auto v = m_rx.pop();
        if (!v) {
            return false;
        }
        b = *v;
        return true;
    }

    /// Queue as much of the buffer as fits; returns the number queued.
    static uint8_t write(const uint8_t* buffer, uint8_t len) {
        uint8_t written = 0;
        while (written < len && write_byte(buffer[written])) {
            ++written;
        }
        return written;
    }

    /// Nothing queued and the shift register empty: what a program waits
    /// on before a reset or a clock change.
    static bool tx_idle() {
        if constexpr (has_tx_engine) {
            if (TxEngine::busy()) {
                return false;
            }
        }
        return m_tx.empty() && (regs().STATR & usart_tc) != 0u;
    }

    // ---- introspection ----------------------------------------------------

    static auto rx_pending() { return m_rx.count(); }

    static uint32_t baud() { return m_baud; }
    static uint32_t actual_baud(uint32_t pclk) { return Resource::actual_baud(pclk); }
    static uint16_t rx_overruns() { return m_rx_overruns; }
    static uint16_t hw_overruns() { return m_hw_overruns; }
    static uint16_t frame_errors() { return m_frame_errors; }
    static uint16_t noise_errors() { return m_noise_errors; }
    static uint16_t parity_errors() { return m_parity_errors; }

    static void clear_errors() {
        m_rx_overruns = 0;
        m_hw_overruns = 0;
        m_frame_errors = 0;
        m_noise_errors = 0;
        m_parity_errors = 0;
        m_dma_faults = 0;
    }

    /// The divisor this bus clock and baud ask for, and the two
    /// questions a program asks before it moves a rate.
    static constexpr uint32_t divisor_for(uint32_t pclk, uint32_t baud) {
        return usart_divisor(pclk, baud);
    }
    static constexpr uint32_t min_hz_for(uint32_t baud) { return usart_min_hz(baud); }
    static constexpr bool can_baud(uint32_t pclk, uint32_t baud) {
        return usart_divisor_valid(usart_divisor(pclk, baud));
    }

    /// Follow a clock that changed rate. The argument is HCLK - what a
    /// dynamic clock hands every user - and this port derives its own
    /// bus rate from it, because the two peripheral buses do not divide
    /// the same number.
    ///
    /// It is called BEFORE the rate moves, with the old clock still
    /// running, so the queue is drained here: a byte still in the shift
    /// register when SYSCLK changes goes out at neither baud rate. The
    /// drain is bounded - a port whose line is held off must not hang
    /// the switch - and the divisor is written whatever the drain found.
    static void rebase(uint32_t hz) {
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
        const uint32_t brr = usart_divisor(usart_bus_hz_at<instance>(hz), m_baud);
        if (usart_divisor_valid(brr)) {
            regs().BRR = static_cast<uint16_t>(brr);
        }
    }

    /// Move the LINK to a different bit rate, the clock staying put -
    /// the mirror of rebase(), and `hz` is HCLK exactly as rebase()
    /// takes it, this port deriving its own bus rate from it. False,
    /// and nothing written, when the rate is unreachable.
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const uint32_t brr = usart_divisor(usart_bus_hz_at<instance>(hz), baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }
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
        regs().BRR = static_cast<uint16_t>(brr);
        m_baud = baud;
        return true;
    }

    /// Stop the port and park its pads: the vector off, the engines
    /// stopped, UE clear, the bus clock closed, the pins released.
    static void release() {
        Pfic::disable(usart_irq_for(instance));
        if constexpr (has_tx_engine) {
            TxEngine::stop();
        }
        if constexpr (has_rx_engine) {
            RxEngine::stop();
        }
        Resource::enable(false);
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
    /// The flow-control pads, formed only where the options ask for them
    /// - a column that has no such pad would not make a Pin at all.
    using Cts = Pin<opts.cts ? column.cts.port : pads.tx.port,
                    opts.cts ? column.cts.pin : pads.tx.pin>;
    using Rts = Pin<opts.rts ? column.rts.port : pads.tx.port,
                    opts.rts ? column.rts.pin : pads.tx.pin>;

    /// Start the next contiguous run of the TX ring on the engine, if it
    /// is idle and there is one. Under the guard: the completion path
    /// runs in the channel's handler.
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            typename P::CriticalSection cs;
            if (TxEngine::busy()) {
                return;
            }
            const auto run = m_tx.read_span();
            if (run.empty()) {
                return;
            }
            (void)TxEngine::start(run.data(), static_cast<uint16_t>(run.size()));
        }
    }

    /// Point the receive engine at the ring's next free run. No room is
    /// a byte lost before it arrives, counted as the software overrun
    /// it is.
    static void rearm_rx() {
        if constexpr (has_rx_engine) {
            const auto room = m_rx.write_span();
            if (room.empty()) {
                bump(m_rx_overruns);
                return;
            }
            (void)RxEngine::start(room.data(), static_cast<uint16_t>(room.size()));
        }
    }

    /// Saturating: a counter that wraps would report a healthy port.
    static void bump(uint16_t& counter) {
        if (counter != 0xFFFFu) {
            ++counter;
        }
    }

    static inline Ring<uint8_t, rx_size, P> m_rx{};
    static inline Ring<uint8_t, tx_size, P> m_tx{};
    static inline uint32_t m_baud = 0;
    static inline uint16_t m_rx_overruns = 0;
    static inline uint16_t m_hw_overruns = 0;
    static inline uint16_t m_frame_errors = 0;
    static inline uint16_t m_noise_errors = 0;
    static inline uint16_t m_parity_errors = 0;
    static inline uint16_t m_dma_faults = 0;
};

} // namespace brio
