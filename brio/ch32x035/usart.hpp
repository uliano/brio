/*
 * usart.hpp
 *
 * The serial ports of the CH32X035 (RM ch. 14) in the two strata every
 * brio serial driver has (docs/design/serial.md):
 *
 *  Usart<n>              the RESOURCE: which instance, where its
 *                        registers are, its bus gate and its reset, its
 *                        vector, its DMA channels, and the whole register
 *                        description of chapter 14 as verbs - the frame,
 *                        the divisor, mute mode with both wakes, LIN's
 *                        break, single-wire half duplex, IrDA, the
 *                        smartcard, the synchronous clock, the
 *                        flow-control pair, the DMA requests, every flag
 *                        and every interrupt enable;
 *  Uart<n, P, ...>       the TASK: the interrupt-driven byte transport with
 *                        two rings and one ISR body - every console's
 *                        personality, a ByteTransport for print() and
 *                        SerialPort, the same surface - and the same
 *                        template parameters - as the CH32V203's
 *                        (brio/ch32vx03/usart.hpp).
 *
 * FOUR INSTANCES, ONE CLOCK. USART1 answers on PB2 and USART2..USART4 on
 * PB1, but this series has no peripheral-bus prescaler: every one of them
 * counts its divisor in HCLK (14.3), so the transport's rate is the
 * clock's and nothing else. WHICH instances a part offers is the part's
 * table (device::has_usart), and on the 20-pin packages that is not the
 * first n of them: the CH32X035F8U6 offers USART2, USART3 and USART4 and
 * no USART1, because no USART1 column has both its TX and its RX pads on
 * that package.
 *
 * EVERY INSTANCE IS A FULL USART. Chapter 14 counts four universal
 * synchronous asynchronous transceivers and the block diagram gives each
 * one RX, TX, CTS, RTS and CK - there is no UART here, so the smartcard,
 * the synchronous clock and the flow-control pair are verbs of every
 * instance. What a COLUMN may lack is a pad: USART3's code 3 has no CK.
 *
 * THE PADS ARE THE REMAP COLUMNS' (afio.hpp): the `remap` template
 * parameter names a COLUMN and the TX, RX, CK, CTS and RTS pads follow
 * from the table that programs AFIO_PCFR1. A code of 0 writes nothing at
 * all - the reset column is already there. Two rules come from the
 * package: a column is a transport's only where its TX pad may be DRIVEN
 * and its RX pad is bonded - a TX on a pad that shares its package pin
 * with another (pin.hpp) is refused, which is what closes USART4's
 * PC16/PC17 columns on every part but the CH32X035F8U6 - and a column
 * whose TX or RX lands on the debug port's own pads (USART3's code 1, and
 * USART4's codes 4 and 6 for RX) is refused by init() while the two-wire
 * port is still the probe's.
 *
 * TWO RINGS AND TWO FLAGS. Bytes leave through a TX ring drained by the
 * TXE interrupt (write_byte() arms TXEIE; the ISR disarms it when the
 * ring runs dry, because TXE stands for ever while the transmitter is
 * idle and would re-enter the handler without end), and arrive into an
 * RX ring filled by RXNE. isr() returns the "RX went non-empty" EDGE that
 * util/serial_port.hpp posts on - one event per idle-to-busy transition,
 * not one per byte.
 *
 * THE BAUD DIVISOR IS THE WHOLE REGISTER. BRR counts HCLK periods per bit
 * in sixteenths (14.3, 14.10.3: a 12-bit mantissa and a 4-bit fraction),
 * so the value to store is simply hclk / baud - no field arithmetic - and
 * actual_baud() inverts it. Below 16 the generator has nothing to divide
 * by: refused.
 *
 * ERRORS ARE READ THEN CLEARED. Parity, framing, noise, overrun and the
 * idle line stand in STATR and go away when STATR is read and then DATAR,
 * in that order (14.10.1); RXNE, TC, LBD and CTS also clear by writing a
 * zero over them. The handler therefore reads the status ONCE, decides
 * from that copy, and lets the DATAR read do the clearing. A byte that
 * arrived with an error is DROPPED rather than pushed, and the counter
 * says it happened.
 *
 * THE MODES EXCLUDE EACH OTHER THE WAY 14.4 .. 14.7 SAY, and the
 * exclusions are NOT symmetrical: the synchronous clock wants SCEN, HDSEL
 * and IREN clear; half duplex wants SCEN, CLKEN and IREN clear; the
 * smartcard wants LINEN, HDSEL and IREN clear and KEEPS CLKEN, which is
 * where its card clock comes from; IrDA wants LINEN, STOP, CLKEN, SCEN and
 * HDSEL clear. Each verb refuses instead of storing a combination the
 * chapter declares undefined.
 *
 * HALF DUPLEX DRIVES PUSH-PULL. 14.5 asks for the TX pad "in output mode
 * plus pull" and table 8-3 for an alternate PUSH-PULL output with an
 * external pull - the only alternate output this series has (pin.hpp).
 * Two talkers on one wire can therefore fight; the manual leaves avoiding
 * that to the program.
 *
 * THE TWO ENGINE SLOTS are the other strata's, and the DMA chapter is
 * not written in this stratum (docs/ch32x035/README.md's gap list): the
 * slots take the empty tag alone (dma_engine.hpp, which also holds the
 * channel each direction requests on), dma_isr() and harvest() answer
 * false and cost nothing, and init() never touches CTLR3's DMA bits.
 */

#pragma once

#include <stdint.h>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/device.hpp"
#include "ch32x035/dma_engine.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
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
/// chapter's own (14.10.5).
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

/// Mute mode (14.10.4's RWU and WAKE, 14.10.5's ADD): the receiver asleep
/// until the line goes idle, or until a frame whose MSB is set carries
/// this node's 4-bit address.
enum class MuteWake : uint8_t { idle_line, address_mark };

struct MuteConfig {
    MuteWake wake = MuteWake::idle_line;
    uint8_t address = 0;   ///< 0..15, for address_mark
};

constexpr bool mute_valid(const MuteConfig& c) { return c.address <= 15u; }

/// LIN mode (14.10.5's LINEN): the break SBK sends, and its detection at
/// ten or eleven bits with its own flag and interrupt.
struct LinConfig {
    bool break_11bit = false;   ///< LBDL: 11-bit detection instead of 10
    bool break_interrupt = false;
};

/// IrDA (14.7, 14.10.7): the SIR encoder on TX and decoder on RX, normal
/// mode at the bit rate or low-power mode on the prescaled clock. GPR.PSC
/// is all eight bits in low-power mode and must be 1 in normal mode; a
/// prescaler of 0 "indicates a hold" and is refused.
struct IrdaConfig {
    bool low_power = false;
    uint8_t prescaler = 1;   ///< GPR.PSC
};

constexpr bool irda_valid(const IrdaConfig& c) {
    return c.prescaler != 0u && (c.low_power || c.prescaler == 1u);
}

/// The synchronous mode's three choices (14.4): the clock's idle level
/// (CPOL), the capture edge (CPHA) and whether the last data bit gets a
/// clock pulse too (LBCL). THE SENSE OF LBCL IS THIS MANUAL'S, and it is
/// the opposite of the F1 family's: 14.10.5 reads "1: The clock pulse for
/// the last bit of data is not output from CK; 0: ... will be output".
/// `last_bit_clock` names the BIT and not a promise.
struct UsartSyncConfig {
    bool clock_idle_high = false;      ///< CPOL
    bool capture_second_edge = false;  ///< CPHA
    bool last_bit_clock = false;       ///< LBCL, as the register spells it
};

/// The smartcard (14.6): ISO 7816-3 on a single wire with the card's clock
/// on CK - a NACK on a parity error, the guard time in bit times (GPR.GT)
/// and the clock prescaler (GPR.PSC, the low FIVE bits, the source divided
/// by twice the value, so 31 is the deepest division and 0 is reserved).
struct SmartcardConfig {
    bool nack = true;
    uint8_t guard_time = 0;
    uint8_t clock_prescaler = 1;
};

constexpr bool smartcard_valid(const SmartcardConfig& c) {
    return c.clock_prescaler != 0u && c.clock_prescaler <= 31u;
}

/// The divisor this clock and baud ask for: hclk/baud in sixteenths,
/// rounded to nearest so the error is halved. Below 16 there is no whole
/// clock period per sixteenth of a bit and the generator has nothing to
/// divide by - init() refuses such a value.
constexpr uint32_t usart_divisor(uint32_t hclk, uint32_t baud) {
    return baud == 0u ? 0u : (hclk + baud / 2u) / baud;
}

constexpr bool usart_divisor_valid(uint32_t brr) { return brr >= 16u && brr <= 0xFFFFu; }

/// The rate a divisor really gives at this clock, and the smallest clock
/// that can produce a rate at all (sixteen periods a bit).
constexpr uint32_t usart_actual_baud(uint32_t hclk, uint32_t brr) {
    return brr == 0u ? 0u : hclk / brr;
}
constexpr uint32_t usart_min_hz(uint32_t baud) { return baud * 16u; }

/// Which bus an instance answers on, and therefore which gate opens it:
/// USART1 alone is PB2's. The divisor counts HCLK on both.
constexpr Bus usart_bus_for(uint8_t n) { return n == 1 ? Bus::pb2 : Bus::pb1; }

/// The gate of an instance (RCC_APB2PCENR bit 14, RCC_APB1PCENR bits
/// 17..19 - 3.4.6, 3.4.7), the same bit in the matching reset register.
constexpr uint32_t usart_gate_for(uint8_t n) {
    return n == 1 ? rcc_pb2_usart1 :
           n == 2 ? rcc_pb1_usart2 :
           n == 3 ? rcc_pb1_usart3 :
           n == 4 ? rcc_pb1_usart4 : 0;
}

/// The instance's vector (RM table 7-1).
constexpr Irq usart_irq_for(uint8_t n) {
    return n == 1 ? Irq::usart1 : n == 2 ? Irq::usart2 : n == 3 ? Irq::usart3 : Irq::usart4;
}

/// The AFIO field an instance's column is selected by.
constexpr Remap usart_remap_of(uint8_t n) { return afio_usart_remap(n); }

/// The whole column - TX, RX, CK, CTS, RTS - under a remap code. ONE
/// table answers, afio.hpp's, which is also what writes the register.
constexpr UsartPadSet usart_column_for(uint8_t n, uint8_t code = 0) {
    return afio_usart_pads(n, code);
}

/// The two pads a transport claims, out of the five a column carries.
struct UsartPads {
    Pad tx;
    Pad rx;
};

constexpr UsartPads usart_pads_for(uint8_t n, uint8_t code = 0) {
    const UsartPadSet p = usart_column_for(n, code);
    return UsartPads{p.tx, p.rx};
}

/// Whether a column is a TRANSPORT's on this part: the code is one of the
/// field's, its TX pad may be driven (bonded and alone on its pin) and
/// its RX pad is bonded.
constexpr bool usart_remap_valid(uint8_t n, uint8_t code) {
    const UsartPadSet p = usart_column_for(n, code);
    return afio_remap_has_code(usart_remap_of(n), code) && pad_can_drive(p.tx) && pad_bonded(p.rx);
}

/// Whether a column puts TX or RX on one of the two pads the debug port
/// owns from reset (device.hpp's debug_swdio_* and debug_swclk_*).
constexpr bool usart_column_on_debug_port(uint8_t n, uint8_t code) {
    const UsartPadSet p = usart_column_for(n, code);
    const Pad dio{debug_swdio_port, debug_swdio_pin};
    const Pad clk{debug_swclk_port, debug_swclk_pin};
    return p.tx == dio || p.tx == clk || p.rx == dio || p.rx == clk;
}

// =============================================================================
// Usart<n>: the resource
// =============================================================================

/**
 * One USART instance, register by register. Every verb stores what it
 * names and nothing else; a verb whose combination the chapter declares
 * undefined refuses (false, nothing written). The task below is written
 * on these verbs; a program that wants the chapter beyond the transport -
 * a break on a LIN line, a muted receiver, a clocked frame - reaches them
 * here.
 */
template <uint8_t n>
struct Usart {
    static_assert(n >= 1u && n <= 4u,
                  "brio Usart: this series has USART1..USART4");
    static_assert(device::has_usart(n),
                  "brio Usart: this part does not offer that instance - its package bonds no "
                  "column of it a transport could use (parts/<part>.hpp)");

    Usart() = delete;

    static constexpr uint8_t number = n;
    static constexpr Bus bus = usart_bus_for(n);
    static constexpr UsartPads pads = usart_pads_for(n);
    static constexpr Irq irq = usart_irq_for(n);
    static constexpr uint8_t dma_tx_channel = usart_dma_tx_channel(n);
    static constexpr uint8_t dma_rx_channel = usart_dma_rx_channel(n);

    /// Every instance of this series is a full USART (the file header).
    static constexpr bool is_full = true;

    static UsartRegs& regs() { return *reinterpret_cast<UsartRegs*>(usart_base_for(n)); }

    /// The peripheral's clock gate. Nothing in this file reads a register
    /// before this is on: an unclocked block answers rubbish.
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
    /// column and writing it is still a write, which is why the transport
    /// skips the call entirely at that code.
    static bool remap(uint8_t code) { return Afio::remap(usart_remap_of(n), code); }

    // ---- configuration ----------------------------------------------------

    /// The frame and the divisor, with the port DISABLED. The two registers
    /// are written WHOLE - this is the from-scratch verb, and the modes
    /// below are what a program adds to it afterwards.
    static bool configure(const UartFormat& f, uint32_t brr) {
        if (!uart_format_valid(f) || !usart_divisor_valid(brr)) {
            return false;
        }
        regs().CTLR1 = usart_ctlr1_format(f);
        regs().CTLR2 = usart_ctlr2_stop(f.stop);
        regs().BRR = static_cast<uint16_t>(brr);
        return true;
    }

    /// The stop bits alone. Refused under IrDA, which 14.7 wants at one.
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

    static void enable(bool on) { bit(regs().CTLR1, usart_ue, on); }
    static void transmitter(bool on) { bit(regs().CTLR1, usart_te, on); }
    static void receiver(bool on) { bit(regs().CTLR1, usart_re, on); }

    // ---- the receiver's modes ---------------------------------------------

    /// Mute mode: WAKE and ADD written, RWU left to mute()/unmute() because
    /// 14.10.4's note 1 says the receiver must have taken a byte before an
    /// idle-line wake can work.
    static bool mute_mode(const MuteConfig& c) {
        if (!mute_valid(c)) {
            return false;
        }
        bit(regs().CTLR1, usart_wake, c.wake == MuteWake::address_mark);
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & ~usart_add_mask) | c.address);
        return true;
    }
    /// Put the receiver to sleep. 14.10.4's note 2: under an address-mark
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
    /// clock is on (14.4 .. 14.7).
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

    /// SBK: one break frame after the current one; the bit clears itself on
    /// the break frame's stop bit (14.10.4).
    static void send_break() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_sbk); }
    static bool break_pending() { return (regs().CTLR1 & usart_sbk) != 0u; }

    /// Single-wire half duplex (14.5): the TX pad alone carries both
    /// directions - an alternate push-pull output with an external pull
    /// (the file header). Refused while LIN, IrDA, the smartcard or the
    /// clock is on.
    static bool half_duplex(bool on) {
        if (on && ((regs().CTLR2 & (usart_linen | usart_clken)) != 0u ||
                   (regs().CTLR3 & (usart_iren | usart_scen)) != 0u)) {
            return false;
        }
        bit(regs().CTLR3, usart_hdsel, on);
        return true;
    }
    static bool half_duplex() { return (regs().CTLR3 & usart_hdsel) != 0u; }

    /// IrDA (14.7): refused while LIN, half duplex, the smartcard or the
    /// clock is on, with stop bits other than one, and with a prescaler
    /// the register description calls a hold.
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

    /// The smartcard (14.6). Sets the guard time and the card-clock
    /// prescaler, 1.5 stop bits, CLKEN and SCEN: the clock is the CARD'S
    /// and belongs to the mode. What is not written is the CK pad - the
    /// caller hands that over (usart_column_for(n, code).ck) - and CPOL,
    /// CPHA and LBCL. Refused while LIN, half duplex or IrDA is on.
    static bool smartcard(const SmartcardConfig& c) {
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
    /// SCEN and its NACK dropped. The CARD CLOCK IS LEFT RUNNING: CLKEN is
    /// the synchronous half's bit and synchronous_off() is what stops it.
    static void smartcard_off() {
        regs().CTLR3 = static_cast<uint16_t>(regs().CTLR3 & ~(usart_scen | usart_nack));
    }
    static bool smartcard_enabled() { return (regs().CTLR3 & usart_scen) != 0u; }

    /// THE SYNCHRONOUS MODE (14.4): the column's CK pad carries a clock
    /// while the TRANSMITTER shifts and at no other time, and this side is
    /// the master. Refused while LIN, half duplex, IrDA or the smartcard is
    /// on, and while the transmitter or the receiver is enabled, because
    /// CPOL, CPHA and LBCL "need to be set when TE and RE are not enabled".
    /// The CK pad itself is the caller's to hand to the peripheral.
    static bool synchronous(const UsartSyncConfig& c) {
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
    /// CLKEN and its three companions dropped; the same TE/RE rule.
    static bool synchronous_off() {
        if ((regs().CTLR1 & (usart_te | usart_re)) != 0u) {
            return false;
        }
        regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 &
                                             ~(usart_clken | usart_cpol | usart_cpha | usart_lbcl));
        return true;
    }
    static bool synchronous_enabled() { return (regs().CTLR2 & usart_clken) != 0u; }

    /// The hardware flow-control pair (14.10.6): RTS driven low while the
    /// receiver can take a frame, CTS sampled before each frame goes out.
    static bool flow_control(bool rts, bool cts) {
        bit(regs().CTLR3, usart_rtse, rts);
        bit(regs().CTLR3, usart_ctse, cts);
        return true;
    }
    static bool rts_enabled() { return (regs().CTLR3 & usart_rtse) != 0u; }
    static bool cts_enabled() { return (regs().CTLR3 & usart_ctse) != 0u; }

    /// CTLR3's two DMA request enables (14.10.6, 14.8). With DMAT set, TXE
    /// raises a request instead of feeding the interrupt; with DMAR set,
    /// so does RXNE.
    static void dma_transmit(bool on) { bit(regs().CTLR3, usart_dmat, on); }
    static void dma_receive(bool on) { bit(regs().CTLR3, usart_dmar, on); }

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
    /// EIE: FE, ORE and NE raise the vector - under DMAR only (14.10.6).
    static void error_interrupt(bool on) { bit(regs().CTLR3, usart_eie, on); }

    static uint16_t status() { return regs().STATR; }
    static bool flag(uint16_t mask) { return (regs().STATR & mask) != 0u; }
    /// The write-zero-to-clear flags (RXNE, TC, LBD, CTS); any other bit in
    /// the mask is ignored, because writing it does nothing.
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

    /// The word as the receiver has it - nine bits when the frame is nine
    /// bits wide. Reading DATAR is what clears RXNE, and the STATR-then-
    /// DATAR pair is what clears the error flags.
    static uint16_t read_word() { return static_cast<uint16_t>(regs().DATAR); }
    static void write_word(uint16_t w) { regs().DATAR = w; }
    static uint8_t read_data() { return static_cast<uint8_t>(regs().DATAR & 0xFFu); }
    static void write_data(uint8_t b) { regs().DATAR = b; }

    /// Read the errors and clear them, the sequence the chapter prescribes
    /// (14.10.1): the status register, then the data one.
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
    static uint32_t actual_baud(uint32_t hclk) {
        const uint32_t brr = regs().BRR;
        return brr == 0u ? 0u : hclk / brr;
    }

private:
    static void bit(volatile uint16_t& r, uint16_t mask, bool on) {
        r = static_cast<uint16_t>(on ? (r | mask) : (r & ~mask));
    }
};

// =============================================================================
// Uart: the transport task
// =============================================================================

/**
 * What a Uart may be told beyond its frame, its rings and its engines, as
 * one trailing template parameter - the CH32V203 stratum's struct, whose
 * defaults are the console's personality.
 */
struct UartOptions {
    /// Single-wire half duplex (14.5): one pad, the TX pad, as an alternate
    /// push-pull output against an external pull; the RX pad is left alone.
    bool half_duplex = false;
    /// The flow-control pair on the column's CTS and RTS pads.
    bool rts = false;
    bool cts = false;
};

/**
 * The interrupt-driven serial port.
 *
 *   using Serial = brio::Uart<2, Platform>;
 *   constexpr Serial serial;                 // tag for print(serial, ...)
 *   Serial::init(clock, 115200);
 *   extern "C" BRIO_CH32_INTERRUPT void usart2_handler() { Serial::isr(); }
 *
 * `P` is the platform, which the rings need to know whether an index can
 * be shared with a handler bare (atomic_width) or wants a guard.
 */
template <uint8_t instance, typename P, uint16_t rx_size = 64, uint16_t tx_size = 64,
          UartFormat format = {}, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine, uint8_t remap = 0, UartOptions opts = {}>
struct Uart {
    static_assert(instance >= 1u && instance <= 4u,
                  "brio Uart: this series has USART1..USART4");
    static_assert(device::has_usart(instance),
                  "brio Uart: this part does not offer that instance (parts/<part>.hpp)");
    static_assert(uart_format_valid(format) && format.bits != UartBits::nine,
                  "brio Uart: the rings carry bytes - seven data bits with a parity bit, or eight, "
                  "with or without one; nine-bit words are the resource's read_word()/write_word()");
    static_assert(usart_remap_valid(instance, remap),
                  "brio Uart: no such column for this instance on this part - the code does not "
                  "fit the field, the package does not bond its RX pad, or its TX pad is not one "
                  "a program may drive (afio.hpp's columns, pin.hpp's shorts, parts/<part>.hpp)");
    static_assert(!opts.rts || pad_can_drive(usart_column_for(instance, remap).rts),
                  "brio Uart: this column's RTS pad is not bonded on this package, or shares "
                  "its pin with another pad (parts/<part>.hpp)");
    static_assert(!opts.cts || pad_bonded(usart_column_for(instance, remap).cts),
                  "brio Uart: this package does not bond this column's CTS pad (parts/<part>.hpp)");
    // An engine is checked where it is NAMED: sizeof demands a complete
    // type, so an engine's own static_asserts fire on the line the
    // application wrote.
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: NoDmaEngine (the default)");
    static_assert(!TxEngine::present && !RxEngine::present,
                  "brio Uart: this stratum has no DMA controller driver, so the engine slots take "
                  "NoDmaEngine alone (ch32x035/dma_engine.hpp)");

    Uart() = default;   // a tag instance: constexpr Uart<2, P> serial;

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
     * Open the port at `baud` in the given frame (8N1 by default), with the
     * receive interrupt armed.
     *
     * Returns false when the divisor comes out below the smallest the
     * chapter allows (16), and when the column lands on the debug port's
     * pads while the two-wire port is still the probe's - a baud rate this
     * clock cannot serve, or a column that would fight the link that put
     * the image there, is a fact the caller must see.
     */
    template <typename C>
    static bool init(C clock, uint32_t baud) {
        static_assert(clock_follows<C, Uart>(),
                      "brio Uart: the baud divisor is derived from HCLK, so a dynamic clock must "
                      "list this port among the users it rebases");

        const uint32_t brr = usart_divisor(clock_hz(clock), baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }

        if constexpr (usart_column_on_debug_port(instance, remap)) {
            if (Afio::debug_port_enabled()) {
                return false;
            }
        }

        Resource::bus_clock(true);

        // The column, only where it is not the reset one: AFIO's reset
        // value already selects code 0.
        if constexpr (remap != 0u) {
            (void)Resource::remap(remap);
        }

        // Pads before the enable: TE's idle frame must land on a pad the
        // peripheral already owns.
        Tx::function();
        if constexpr (!opts.half_duplex) {
            (void)Rx::input();                   // floating: the peer drives it
        }
        if constexpr (opts.rts) {
            Rts::function();
        }
        if constexpr (opts.cts) {
            (void)Cts::input(PinPull::up);       // unconnected reads "not clear to send"
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

        // CTLR3 IS TOUCHED ONLY BY A PORT THAT HAS SOMETHING TO PUT IN IT:
        // with no option there is nothing of this transport's there, and
        // whatever a program stored itself stays.
        if constexpr (opts.half_duplex || opts.rts || opts.cts) {
            uint16_t ctlr3 = regs().CTLR3;
            if constexpr (opts.half_duplex) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_hdsel); }
            if constexpr (opts.rts) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_rtse); }
            if constexpr (opts.cts) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_ctse); }
            regs().CTLR3 = ctlr3;
        }

        regs().CTLR1 = static_cast<uint16_t>(usart_ctlr1_format(format) | usart_ue | usart_te |
                                             usart_re | usart_rxneie);

        Pfic::enable(usart_irq_for(instance));
        return true;
    }

    /// The ISR body of a DMA channel this transport owns: false and free,
    /// with the slots empty (the file header).
    [[gnu::always_inline]] static bool dma_isr() { return false; }

    /// Ask the receive engine what has arrived: false and free, with no
    /// engine.
    static bool harvest() { return false; }

    /// Blocks the engines threw away: 0, with no engine.
    static uint16_t dma_faults() { return 0; }

    /**
     * The port's whole interrupt body - bind this instance's vector to it.
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
                // TXE stands while the transmitter is idle: disarm, or the
                // handler is re-entered for ever.
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
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_txeie);
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

    /// Nothing queued and the shift register empty: what a program waits on
    /// before a reset or a clock change.
    static bool tx_idle() { return m_tx.empty() && (regs().STATR & usart_tc) != 0u; }

    // ---- introspection ----------------------------------------------------

    static auto rx_pending() { return m_rx.count(); }

    static uint32_t baud() { return m_baud; }
    static uint32_t actual_baud(uint32_t hclk) { return Resource::actual_baud(hclk); }
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
    }

    /// The divisor this clock and baud ask for, and the two questions a
    /// program asks before it moves a rate.
    static constexpr uint32_t divisor_for(uint32_t hclk, uint32_t baud) {
        return usart_divisor(hclk, baud);
    }
    static constexpr uint32_t min_hz_for(uint32_t baud) { return usart_min_hz(baud); }
    static constexpr bool can_baud(uint32_t hclk, uint32_t baud) {
        return usart_divisor_valid(usart_divisor(hclk, baud));
    }

    /// Follow a clock that changed rate. The argument is HCLK, which is
    /// what the divisor counts on this series.
    ///
    /// It is called BEFORE the rate moves, with the old clock still
    /// running, so the queue is drained here: a byte still in the shift
    /// register when HCLK changes goes out at neither baud rate. The drain
    /// is bounded - a port whose line is held off must not hang the switch
    /// - and the divisor is written whatever the drain found.
    static void rebase(uint32_t hz) {
        constexpr uint32_t drain_spins = 2'000'000UL;
        uint32_t spins = drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = drain_spins;
        while ((regs().STATR & usart_tc) == 0u && spins-- != 0u) {
        }
        const uint32_t brr = usart_divisor(hz, m_baud);
        if (usart_divisor_valid(brr)) {
            regs().BRR = static_cast<uint16_t>(brr);
        }
    }

    /// Move the LINK to a different bit rate, the clock staying put - the
    /// mirror of rebase(), `hz` being HCLK as rebase() takes it. False, and
    /// nothing written, when the rate is unreachable.
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const uint32_t brr = usart_divisor(hz, baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }
        constexpr uint32_t drain_spins = 2'000'000UL;
        uint32_t spins = drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = drain_spins;
        while ((regs().STATR & usart_tc) == 0u && spins-- != 0u) {
        }
        regs().BRR = static_cast<uint16_t>(brr);
        m_baud = baud;
        return true;
    }

    /// Stop the port and park its pads: the vector off, UE clear, the bus
    /// clock closed, the pins released.
    static void release() {
        Pfic::disable(usart_irq_for(instance));
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
    /// The flow-control pads, formed only where the options ask for them -
    /// a column that has no such pad would not make a Pin at all.
    using Cts = Pin<opts.cts ? column.cts.port : pads.tx.port,
                    opts.cts ? column.cts.pin : pads.tx.pin>;
    using Rts = Pin<opts.rts ? column.rts.port : pads.tx.port,
                    opts.rts ? column.rts.pin : pads.tx.pin>;

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
};

} // namespace brio
