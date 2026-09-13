/*
 * usart.hpp
 *
 * The USART (RM0090 ch. 30, RM0390 ch. 25, RM0383 ch. 19 - one chapter,
 * three manuals, the same registers) in the two strata every brio
 * serial driver has (docs/design/serial.md):
 *
 *  Usart<n>              the RESOURCE: which instance (USART1..3, UART4/5,
 *                        USART6, UART7/8 - the family's numbering), where
 *                        its registers are, its bus and its gate, its
 *                        vector, and the whole register description of
 *                        the chapter as verbs - the frame, the divisor
 *                        in either oversampling, mute mode with both
 *                        wakes, LIN's break, single-wire half duplex,
 *                        IrDA, the smartcard, the synchronous clock, the
 *                        flow-control pair, the DMA requests, every flag
 *                        and every interrupt enable;
 *  Uart<n, pins, ...>    the TASK: the interrupt-driven byte transport
 *                        with two rings and one ISR body - every
 *                        console's personality, a ByteTransport for
 *                        print() and SerialPort, the same surface the
 *                        other five targets expose. Its OPTIONS ride one
 *                        trailing template parameter and cost nothing
 *                        when left at their defaults.
 *
 * THE F1 LINEAGE. This is the classic STM32 USART - SR/DR/BRR/CR1/CR2/
 * CR3/GTPR - the block the CH32V00x stratum drives under WCH's names,
 * and NOT the STM32G0's (ISR/ICR/TDR/RDR, FIFOs, a prescaler, a
 * wake-from-Stop unit). What that means for the code: the flags are
 * cleared by READ SEQUENCES and not by a clear register (PE, FE, NE,
 * ORE and IDLE by reading SR then DR - 30.6.1 - TC by reading SR then
 * writing DR; RXNE by reading DR; LBD and CTS by writing 0), and the
 * handler is written around those sequences: it reads SR ONCE, decides
 * from that copy, and lets the DR read do the clearing.
 *
 * TWO RINGS AND TWO FLAGS. Bytes leave through a TX ring drained by
 * the TXE interrupt (write_byte() arms TXEIE; the ISR disarms it when
 * the ring runs dry, because TXE stands for ever while the transmitter
 * is idle and would re-enter the handler without end), and arrive into
 * an RX ring filled by RXNE. isr() returns the "RX went non-empty" EDGE
 * that util/serial_port.hpp wants to post on - one event per idle-to-
 * busy transition, not one per byte.
 *
 * THE BAUD DIVISOR (30.3.4): USARTDIV = fCK / (8 x (2 - OVER8) x baud),
 * a 12-bit mantissa and a 4-bit fraction in BRR. At OVER8 = 0 the
 * register value is simply fCK / baud in sixteenths - no field
 * arithmetic at all - and at OVER8 = 1 the fraction has three bits and
 * bit 3 must stay clear (30.6.3), which usart_brr_over8() lays out.
 * fCK is the instance's APB clock: PCLK2 for USART1 and USART6, PCLK1
 * for the rest (stm32f4/device_tables.hpp), and on this family the two
 * differ from HCLK at every rate above the buses' ceilings - the task's
 * init() asks the clock for the right one (stm32f4/clock.hpp's
 * apb_hz). Below 16 the generator has nothing to divide by: refused.
 *
 * A BYTE THAT ARRIVED WITH AN ERROR IS DROPPED rather than pushed: a
 * corrupted byte in a line assembler is worse than a gap, and the
 * counter says it happened. Noise alone (NE) keeps the byte and counts.
 *
 * FULL AND NOT. UART4, UART5, UART7 and UART8 have no synchronous mode,
 * no smartcard and no hardware flow control (30.5 table 148), and the
 * 0.5 and 1.5 stop bits are not theirs either (30.6.5). Each such verb
 * refuses on them at compile time where the instance is a template
 * argument and at run time where it is not.
 *
 * THE MODES EXCLUDE EACH OTHER THE WAY THE CHAPTER SAYS (30.3.8 .. 30.3.11
 * and each mode's own list): half duplex is refused while LIN, smartcard
 * or IrDA is on; IrDA while LIN, half duplex or smartcard is on or the
 * stop bits are not one; LIN while half duplex, smartcard, IrDA or the
 * clock is on; the smartcard while LIN, half duplex or IrDA is on (it
 * wants the clock and 1.5 stop bits, which it sets itself); the
 * synchronous clock while LIN, half duplex or IrDA is on - each verb
 * refuses instead of storing a combination the chapter declares
 * undefined.
 *
 * TWO OPTIONAL DMA ENGINE SLOTS, the shape every brio Uart has: name a
 * stm32f4/dma.hpp DmaTxEngine and/or DmaRxEngine and the bytes move
 * without the CPU; name neither (the default, stm32f4/dma_engine.hpp's
 * NoDmaEngine) and every engine branch below disappears - `if constexpr`
 * throughout, so an engineless image carries no DMA code at all and is
 * byte-identical to the one built before the slots were filled.
 *
 * A STREAM AND A CHANNEL ARE NOT FREE HERE. On this family's controller
 * a peripheral reaches exactly the one or two (controller, stream,
 * channel) cells the request mapping gives it (RM0090 tables 43 and 44,
 * RM0390 tables 28 and 29, RM0383 tables 27 and 28 - they differ by
 * part), so an engine slot is checked against the reserve's serial
 * slice of those tables at compile time. A part class whose manual was
 * not read has no table, and an engine on it is REFUSED rather than run
 * on a guessed channel.
 *
 * CLEARING A RECEIVE ERROR COSTS A BYTE in DMA reception, and that is
 * this block's lineage speaking: ORE, FE, NE and PE are cleared by
 * reading SR and then DR (30.6.1), and under DMAR the stream is what
 * normally reads DR. harvest() therefore clears only when a flag really
 * stands, and counts the byte the clearing read takes out of the
 * stream's reach.
 *
 * THE PADS ARE THE APPLICATION'S: TX and RX as PinSel with the AF the
 * datasheet gives them (AF7 for USART1..3, AF8 for the rest), CTS and
 * RTS likewise through the options. init() hands them over BEFORE the
 * enable, so TE's idle frame lands on a pad the peripheral already
 * owns; the RX pad gets a pull-up so an unconnected line reads idle.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <span>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/dma_engine.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

// ---- the frame ------------------------------------------------------------------

enum class UartBits : uint8_t { seven = 7, eight = 8, nine = 9 };

enum class UartParity : uint8_t { none, even, odd };

/// CR2.STOP, all four codes (30.6.5); the half and one-and-a-half stops
/// are the FULL instances' alone.
enum class UartStop : uint8_t { one = 0, half = 1, two = 2, one_and_half = 3 };

/// The frame the two ends agree on. Defaults to 8N1.
struct UartFormat {
    UartBits bits = UartBits::eight;
    UartParity parity = UartParity::none;
    UartStop stop = UartStop::one;
};

/// The word length is the data bits plus the parity bit, and M has two
/// values: seven data bits exist only with parity, nine only without.
constexpr bool uart_format_valid(const UartFormat& f) {
    if (f.bits == UartBits::seven) {
        return f.parity != UartParity::none;
    }
    if (f.bits == UartBits::nine) {
        return f.parity == UartParity::none;
    }
    return true;
}

/// CR1's M, PCE and PS for a format.
constexpr uint32_t usart_cr1_format(const UartFormat& f) {
    uint32_t v = 0;
    const bool parity = f.parity != UartParity::none;
    if (f.bits == UartBits::nine || (f.bits == UartBits::eight && parity)) {
        v |= USART_CR1_M;
    }
    if (parity) {
        v |= USART_CR1_PCE;
    }
    if (f.parity == UartParity::odd) {
        v |= USART_CR1_PS;
    }
    return v;
}

constexpr uint32_t usart_cr2_stop(UartStop s) {
    return static_cast<uint32_t>(s) << USART_CR2_STOP_Pos;
}

/// The mask of the data bits a frame carries in DR (the parity bit,
/// when there is one, is the receiver's to check and not the byte's).
constexpr uint16_t uart_data_mask(const UartFormat& f) {
    return f.bits == UartBits::nine ? 0x1FFu : f.bits == UartBits::eight ? 0xFFu : 0x7Fu;
}

// ---- the divisor (30.3.4, 30.6.3) ----------------------------------------------------

/// BRR at OVER8 = 0: fCK / baud in sixteenths, rounded to nearest; 16
/// is the smallest value the generator can divide by and 0xFFFF the
/// register's top. Empty when the rate cannot be produced.
constexpr std::optional<uint16_t> usart_brr(uint32_t hz, uint32_t baud) {
    if (hz == 0u || baud == 0u) {
        return std::nullopt;
    }
    const uint32_t div = (hz + baud / 2u) / baud;
    if (div < 16u || div > 0xFFFFu) {
        return std::nullopt;
    }
    return static_cast<uint16_t>(div);
}

/// BRR at OVER8 = 1: fCK / baud in EIGHTHS, the mantissa in bits 15:4
/// and the three-bit fraction in bits 2:0 with bit 3 clear.
constexpr std::optional<uint16_t> usart_brr_over8(uint32_t hz, uint32_t baud) {
    if (hz == 0u || baud == 0u) {
        return std::nullopt;
    }
    const uint32_t div = ((hz << 1) + baud / 2u) / baud;   // in eighths
    if (div < 16u || div > 0xFFFFu) {
        return std::nullopt;
    }
    return static_cast<uint16_t>(((div >> 3) << 4) | (div & 0x7u));
}

/// The rate a BRR value really gives at fCK, either oversampling.
constexpr uint32_t usart_actual_baud(uint32_t hz, uint16_t brr) {
    return brr == 0u ? 0u : hz / brr;
}
constexpr uint32_t usart_actual_baud_over8(uint32_t hz, uint16_t brr) {
    const uint32_t eighths = (static_cast<uint32_t>(brr >> 4) << 3) | (brr & 0x7u);
    return eighths == 0u ? 0u : (hz << 1) / eighths;
}

/// The smallest fCK that can produce `baud`: sixteen (eight) clocks a bit.
constexpr uint32_t usart_min_hz(uint32_t baud) { return baud * 16u; }
constexpr uint32_t usart_min_hz_over8(uint32_t baud) { return baud * 8u; }

// ---- the modes' vocabulary -------------------------------------------------------------

/// Mute mode (30.3.6): the receiver asleep until the line goes idle, or
/// until a frame whose MSB is set carries this node's 4-bit address.
enum class MuteWake : uint8_t { idle_line, address_mark };

struct MuteConfig {
    MuteWake wake = MuteWake::idle_line;
    uint8_t address = 0;   ///< 0..15, for address_mark (CR2.ADD)
};

constexpr bool mute_valid(const MuteConfig& c) { return c.address <= 15u; }

/// LIN mode (30.3.8): the break of thirteen low bits SBK sends, and its
/// detection at ten or eleven bits with its own flag and interrupt.
struct LinConfig {
    bool break_11bit = false;   ///< LBDL: 11-bit detection instead of 10
    bool break_interrupt = false;
};

/// IrDA (30.3.11): the SIR encoder on TX and decoder on RX, normal mode
/// at the bit rate (a 3/16 pulse) or low-power mode on the prescaled
/// clock (GTPR.PSC, 8 bits). A prescaler of 0 is Reserved and refused.
struct IrdaConfig {
    bool low_power = false;
    uint8_t prescaler = 1;
};

/// The synchronous mode's three choices (30.3.9, figure 316): the
/// clock's idle level (CPOL), the capture edge (CPHA) and whether the
/// last data bit gets a clock pulse too (LBCL).
struct UsartSyncConfig {
    bool clock_idle_high = false;
    bool capture_second_edge = false;
    bool last_bit_clock = false;
};

/// The smartcard (30.3.10): the ISO 7816-3 asynchronous protocol on a
/// single wire with the clock on CK - NACK on a parity error, the guard
/// time in bit times (GTPR.GT) and the clock prescaler (GTPR.PSC, five
/// bits, the source divided by twice the value).
struct SmartcardConfig {
    bool nack = true;
    uint8_t guard_time = 0;
    uint8_t clock_prescaler = 1;   ///< 1..31: fCK / (2 x value) on CK
};

constexpr bool irda_valid(const IrdaConfig& c) { return c.prescaler != 0u; }
constexpr bool smartcard_valid(const SmartcardConfig& c) {
    return c.clock_prescaler >= 1u && c.clock_prescaler <= 31u;
}

/// The SR bits, spelled once for the handlers and the suites.
struct UsartFlag {
    UsartFlag() = delete;
    static constexpr uint32_t pe = USART_SR_PE;
    static constexpr uint32_t fe = USART_SR_FE;
    static constexpr uint32_t ne = USART_SR_NE;
    static constexpr uint32_t ore = USART_SR_ORE;
    static constexpr uint32_t idle = USART_SR_IDLE;
    static constexpr uint32_t rxne = USART_SR_RXNE;
    static constexpr uint32_t tc = USART_SR_TC;
    static constexpr uint32_t txe = USART_SR_TXE;
    static constexpr uint32_t lbd = USART_SR_LBD;
    static constexpr uint32_t cts = USART_SR_CTS;
    /// The four the receiver may attach to a byte.
    static constexpr uint32_t receive_errors = pe | fe | ne | ore;
    /// The write-zero-to-clear ones (30.6.1); every other bit ignores a
    /// write.
    static constexpr uint32_t rc_w0 = rxne | tc | lbd | cts;
};

// =============================================================================
// Usart<n>: the resource
// =============================================================================

/**
 * One serial instance, register by register. Every verb stores what it
 * names and nothing else; a verb whose combination the chapter declares
 * undefined, or whose feature the instance has not got, refuses (false,
 * nothing written). The task below is written on these verbs; a program
 * that wants the chapter beyond the transport - a break on a LIN line,
 * a muted receiver, an IrDA pulse, a card clock - reaches them here.
 */
template <uint8_t n>
struct Usart {
    static_assert(usart_present(n),
                  "brio Usart: this device has no serial instance of that number (the "
                  "device header declares no base for it)");

    Usart() = delete;

    static constexpr uint8_t number = n;
    static constexpr bool on_apb2 = usart_on_apb2(n);
    /// Synchronous mode, smartcard, hardware flow control and the half
    /// stop bits: the FULL instances' (USART1, 2, 3, 6).
    static constexpr bool is_full = usart_is_full(n);
    static constexpr IRQn_Type irq = usart_irq(n);

    static USART_TypeDef& regs() { return *reinterpret_cast<USART_TypeDef*>(usart_base(n)); }

    // ---- the gate and the reset -----------------------------------------------------------
    static void bus_clock(bool on) {
        if constexpr (on_apb2) {
            Rcc::apb2_clock(usart_clock_mask(n), on);
        } else {
            Rcc::apb1_clock(usart_clock_mask(n), on);
        }
    }
    static bool bus_clock() {
        if constexpr (on_apb2) {
            return Rcc::apb2_clock(usart_clock_mask(n));
        } else {
            return Rcc::apb1_clock(usart_clock_mask(n));
        }
    }
    /// Every register to its reset value through the RCC's reset line.
    static void reset() {
        if constexpr (on_apb2) {
            Rcc::apb2_reset(usart_clock_mask(n));
        } else {
            Rcc::apb1_reset(usart_clock_mask(n));
        }
    }

    // ---- enables ---------------------------------------------------------------------------
    static bool enabled() { return (regs().CR1 & USART_CR1_UE) != 0u; }
    /// UE: off, the prescalers and the outputs stop at the end of the
    /// current byte (30.6.4).
    static void enable(bool on) { bit(regs().CR1, USART_CR1_UE, on); }
    static void transmitter(bool on) { bit(regs().CR1, USART_CR1_TE, on); }
    static void receiver(bool on) { bit(regs().CR1, USART_CR1_RE, on); }

    // ---- the frame and the rate -------------------------------------------------------------
    /// Write the frame into CR1 (M, PCE, PS - the rest of the register
    /// kept) and CR2 (STOP), and the divisor into BRR. Refuses a format
    /// the word length cannot carry, a half stop bit on a non-full
    /// instance, and a divisor below 16.
    static bool configure(const UartFormat& f, uint16_t brr) {
        if (!uart_format_valid(f) || brr < 16u) {
            return false;
        }
        if (!is_full && (f.stop == UartStop::half || f.stop == UartStop::one_and_half)) {
            return false;
        }
        USART_TypeDef& r = regs();
        r.CR1 = (r.CR1 & ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS)) | usart_cr1_format(f);
        r.CR2 = (r.CR2 & ~USART_CR2_STOP_Msk) | usart_cr2_stop(f.stop);
        r.BRR = brr;
        return true;
    }
    static bool stop_bits(UartStop s) {
        if (!is_full && (s == UartStop::half || s == UartStop::one_and_half)) {
            return false;
        }
        // 30.3.11: IrDA wants one stop bit.
        if ((regs().CR3 & USART_CR3_IREN) != 0u && s != UartStop::one) {
            return false;
        }
        regs().CR2 = (regs().CR2 & ~USART_CR2_STOP_Msk) | usart_cr2_stop(s);
        return true;
    }
    static bool set_brr(uint16_t v) {
        if (v < 16u) {
            return false;
        }
        regs().BRR = v;
        return true;
    }
    static uint16_t brr() { return static_cast<uint16_t>(regs().BRR); }
    /// OVER8 (30.6.4): eight samples a bit instead of sixteen, twice the
    /// top rate for half the receiver's tolerance. Written with UE clear
    /// only, per the bit's description: refused otherwise.
    static bool oversampling8(bool on) {
        if (enabled()) {
            return false;
        }
        bit(regs().CR1, USART_CR1_OVER8, on);
        return true;
    }
    static bool oversampling8() { return (regs().CR1 & USART_CR1_OVER8) != 0u; }
    /// The rate the divisor in the register gives at this fCK, in the
    /// oversampling the register says.
    static uint32_t actual_baud(uint32_t fck) {
        const uint16_t v = brr();
        return oversampling8() ? usart_actual_baud_over8(fck, v) : usart_actual_baud(fck, v);
    }
    /// ONEBIT (30.6.6): one sample per bit instead of three, more
    /// tolerance to a clock deviation, no noise detection.
    static void one_bit_sampling(bool on) { bit(regs().CR3, USART_CR3_ONEBIT, on); }

    // ---- the modes -------------------------------------------------------------------------------
    /// Mute mode: WAKE and ADD written, RWU left to mute()/unmute()
    /// because 30.3.6 wants the receiver to have seen a byte before an
    /// idle-line wake can work.
    static bool mute_mode(const MuteConfig& c) {
        if (!mute_valid(c)) {
            return false;
        }
        bit(regs().CR1, USART_CR1_WAKE, c.wake == MuteWake::address_mark);
        regs().CR2 = (regs().CR2 & ~USART_CR2_ADD_Msk) | (static_cast<uint32_t>(c.address) << USART_CR2_ADD_Pos);
        return true;
    }
    /// Put the receiver to sleep. 30.6.4's RWU note: under an address-
    /// mark wake RWU must not be set while RXNE stands - refused then.
    static bool mute() {
        if ((regs().CR1 & USART_CR1_WAKE) != 0u && (regs().SR & USART_SR_RXNE) != 0u) {
            return false;
        }
        regs().CR1 |= USART_CR1_RWU;
        return true;
    }
    static void unmute() { regs().CR1 &= ~USART_CR1_RWU; }
    static bool muted() { return (regs().CR1 & USART_CR1_RWU) != 0u; }

    /// LIN mode (30.3.8). Refused while half duplex, smartcard, IrDA or
    /// the clock is on (30.6.5's LINEN: "CLKEN, STOP[1:0], SCEN, HDSEL
    /// and IREN bits must be kept cleared").
    static bool lin(const LinConfig& c) {
        if ((regs().CR3 & (USART_CR3_HDSEL | USART_CR3_IREN | USART_CR3_SCEN)) != 0u ||
            (regs().CR2 & USART_CR2_CLKEN) != 0u) {
            return false;
        }
        uint32_t v = regs().CR2 & ~(USART_CR2_LBDL | USART_CR2_LBDIE);
        v |= USART_CR2_LINEN;
        if (c.break_11bit) { v |= USART_CR2_LBDL; }
        if (c.break_interrupt) { v |= USART_CR2_LBDIE; }
        regs().CR2 = v;
        return true;
    }
    static void lin_off() { regs().CR2 &= ~(USART_CR2_LINEN | USART_CR2_LBDL | USART_CR2_LBDIE); }
    static bool lin_enabled() { return (regs().CR2 & USART_CR2_LINEN) != 0u; }
    /// SBK: one break frame after the current one; the bit clears itself
    /// when the break's stop bit is out. Thirteen bits long in LIN mode,
    /// ten or eleven elsewhere (30.3.2).
    static void send_break() { regs().CR1 |= USART_CR1_SBK; }
    static bool break_pending() { return (regs().CR1 & USART_CR1_SBK) != 0u; }

    /// Single-wire half duplex (30.3.10): TX and RX joined inside the
    /// chip, the RX pad unused. Refused while LIN, smartcard or IrDA is
    /// on, or the clock.
    static bool half_duplex(bool on) {
        if (on && ((regs().CR2 & (USART_CR2_LINEN | USART_CR2_CLKEN)) != 0u ||
                   (regs().CR3 & (USART_CR3_IREN | USART_CR3_SCEN)) != 0u)) {
            return false;
        }
        bit(regs().CR3, USART_CR3_HDSEL, on);
        return true;
    }
    static bool half_duplex() { return (regs().CR3 & USART_CR3_HDSEL) != 0u; }

    /// IrDA (30.3.11): refused while LIN, half duplex or smartcard is on,
    /// with stop bits other than one, or with a prescaler of zero. PSC is
    /// GTPR's low byte.
    static bool irda(const IrdaConfig& c) {
        if (!irda_valid(c)) {
            return false;
        }
        if ((regs().CR2 & USART_CR2_LINEN) != 0u || (regs().CR3 & (USART_CR3_HDSEL | USART_CR3_SCEN)) != 0u) {
            return false;
        }
        if ((regs().CR2 & USART_CR2_STOP_Msk) != 0u) {
            return false;
        }
        regs().GTPR = (regs().GTPR & ~USART_GTPR_PSC_Msk) | c.prescaler;
        uint32_t v = regs().CR3 & ~USART_CR3_IRLP;
        if (c.low_power) { v |= USART_CR3_IRLP; }
        v |= USART_CR3_IREN;
        regs().CR3 = v;
        return true;
    }
    static void irda_off() { regs().CR3 &= ~(USART_CR3_IREN | USART_CR3_IRLP); }
    static bool irda_enabled() { return (regs().CR3 & USART_CR3_IREN) != 0u; }

    /// The smartcard (30.3.10): a FULL instance's. Sets the clock (CLKEN),
    /// 1.5 stop bits and SCEN; refused while LIN, half duplex or IrDA
    /// is on, or with a prescaler outside 1..31. The card's clock is
    /// fCK / (2 x prescaler) on the CK pad, the guard time in bit times.
    static bool smartcard(const SmartcardConfig& c) {
        if constexpr (!is_full) {
            (void)c;
            return false;
        } else {
            if (!smartcard_valid(c)) {
                return false;
            }
            if ((regs().CR2 & USART_CR2_LINEN) != 0u || (regs().CR3 & (USART_CR3_HDSEL | USART_CR3_IREN)) != 0u) {
                return false;
            }
            regs().GTPR = (static_cast<uint32_t>(c.guard_time) << USART_GTPR_GT_Pos) | c.clock_prescaler;
            regs().CR2 = (regs().CR2 & ~USART_CR2_STOP_Msk) | usart_cr2_stop(UartStop::one_and_half) | USART_CR2_CLKEN;
            uint32_t v = regs().CR3 & ~USART_CR3_NACK;
            if (c.nack) { v |= USART_CR3_NACK; }
            v |= USART_CR3_SCEN;
            regs().CR3 = v;
            return true;
        }
    }
    static void smartcard_off() {
        if constexpr (is_full) {
            regs().CR3 &= ~(USART_CR3_SCEN | USART_CR3_NACK);
        }
    }
    static bool smartcard_enabled() {
        if constexpr (!is_full) {
            return false;
        } else {
            return (regs().CR3 & USART_CR3_SCEN) != 0u;
        }
    }

    /// THE SYNCHRONOUS MODE (30.3.9): the CK pad carries a clock while
    /// the transmitter shifts - and only then; the receiver samples on
    /// it. A FULL instance's. Refused while LIN, half duplex, IrDA or
    /// the smartcard is on, and while the transmitter or the receiver is
    /// enabled - CPOL, CPHA and LBCL "should not be written while the
    /// transmitter is enabled", so the caller pauses both around this
    /// verb. The CK pad is the caller's to hand to the peripheral.
    static bool synchronous(const UsartSyncConfig& c) {
        if constexpr (!is_full) {
            (void)c;
            return false;
        } else {
            if ((regs().CR2 & USART_CR2_LINEN) != 0u ||
                (regs().CR3 & (USART_CR3_HDSEL | USART_CR3_IREN | USART_CR3_SCEN)) != 0u) {
                return false;
            }
            if ((regs().CR1 & (USART_CR1_TE | USART_CR1_RE)) != 0u) {
                return false;
            }
            uint32_t v = regs().CR2 & ~(USART_CR2_CPOL | USART_CR2_CPHA | USART_CR2_LBCL);
            if (c.clock_idle_high) { v |= USART_CR2_CPOL; }
            if (c.capture_second_edge) { v |= USART_CR2_CPHA; }
            if (c.last_bit_clock) { v |= USART_CR2_LBCL; }
            v |= USART_CR2_CLKEN;
            regs().CR2 = v;
            return true;
        }
    }
    /// CLKEN and its three companions dropped; the same TE/RE rule.
    static bool synchronous_off() {
        if constexpr (!is_full) {
            return false;
        } else {
            if ((regs().CR1 & (USART_CR1_TE | USART_CR1_RE)) != 0u) {
                return false;
            }
            regs().CR2 &= ~(USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA | USART_CR2_LBCL);
            return true;
        }
    }
    static bool synchronous_enabled() {
        if constexpr (!is_full) {
            return false;
        } else {
            return (regs().CR2 & USART_CR2_CLKEN) != 0u;
        }
    }

    /// The hardware flow-control pair (30.3.12): RTS driven low while
    /// the receiver can take a frame, CTS sampled before each frame goes
    /// out. A FULL instance's.
    static bool flow_control(bool rts, bool cts) {
        if constexpr (!is_full) {
            (void)rts; (void)cts;
            return false;
        } else {
            bit(regs().CR3, USART_CR3_RTSE, rts);
            bit(regs().CR3, USART_CR3_CTSE, cts);
            return true;
        }
    }
    static bool rts_enabled() { return (regs().CR3 & USART_CR3_RTSE) != 0u; }
    static bool cts_enabled() { return (regs().CR3 & USART_CR3_CTSE) != 0u; }

    static void dma_transmit(bool on) { bit(regs().CR3, USART_CR3_DMAT, on); }
    static void dma_receive(bool on) { bit(regs().CR3, USART_CR3_DMAR, on); }

    // ---- interrupts and flags (30.4, table 147) -------------------------------------------------------
    /// CR1's five enables: USART_CR1_PEIE, TXEIE, TCIE, RXNEIE (ORE rides
    /// it), IDLEIE.
    static void interrupts(uint32_t cr1_mask, bool on) { bit(regs().CR1, cr1_mask, on); }
    static void rxne_interrupt(bool on) { bit(regs().CR1, USART_CR1_RXNEIE, on); }
    static void txe_interrupt(bool on) { bit(regs().CR1, USART_CR1_TXEIE, on); }
    static bool txe_interrupt() { return (regs().CR1 & USART_CR1_TXEIE) != 0u; }
    static void tc_interrupt(bool on) { bit(regs().CR1, USART_CR1_TCIE, on); }
    static void idle_interrupt(bool on) { bit(regs().CR1, USART_CR1_IDLEIE, on); }
    static void parity_interrupt(bool on) { bit(regs().CR1, USART_CR1_PEIE, on); }
    static void break_interrupt(bool on) { bit(regs().CR2, USART_CR2_LBDIE, on); }
    static void cts_interrupt(bool on) { bit(regs().CR3, USART_CR3_CTSIE, on); }
    /// EIE: FE, ORE and NE raise the vector - under DMAR only (30.6.6).
    static void error_interrupt(bool on) { bit(regs().CR3, USART_CR3_EIE, on); }

    static uint32_t status() { return regs().SR; }
    static bool flag(uint32_t mask) { return (regs().SR & mask) != 0u; }
    /// The write-zero-to-clear flags (RXNE, TC, LBD, CTS); any other bit
    /// in the mask is ignored, because writing it does nothing.
    static void clear_flags(uint32_t mask) { regs().SR = ~(mask & UsartFlag::rc_w0); }
    /// The read-sequence clear of IDLE, ORE, NE, FE and PE (and of RXNE,
    /// whose byte this discards).
    static void clear_by_read() {
        (void)regs().SR;
        (void)regs().DR;
    }

    // ---- the data register --------------------------------------------------------------------
    static bool tx_empty() { return (regs().SR & USART_SR_TXE) != 0u; }
    static bool tx_complete() { return (regs().SR & USART_SR_TC) != 0u; }
    static bool rx_ready() { return (regs().SR & USART_SR_RXNE) != 0u; }
    static void write_data(uint8_t b) { regs().DR = b; }
    /// A 9-bit word (M set): the ninth bit is data or, with parity on,
    /// the receiver's parity bit.
    static void write_word(uint16_t v) { regs().DR = v & 0x1FFu; }
    static uint8_t read_data() { return static_cast<uint8_t>(regs().DR & 0xFFu); }
    static uint16_t read_word() { return static_cast<uint16_t>(regs().DR & 0x1FFu); }
    static volatile uint32_t* data_address() { return &regs().DR; }

private:
    static void bit(volatile uint32_t& r, uint32_t mask, bool on) {
        if (on) {
            r |= mask;
        } else {
            r &= ~mask;
        }
    }
};

// =============================================================================
// Uart: the transport task
// =============================================================================

/// The two pads a transport claims, each with the AF the datasheet
/// gives the signal on that pad.
struct UartPins {
    PinSel tx;
    PinSel rx;
};

constexpr bool uart_pins_valid(const UartPins& p) {
    return p.tx.valid() && p.rx.valid() &&
           !(p.tx.port == p.rx.port && p.tx.pin == p.rx.pin);
}
constexpr bool uart_pins_valid_single(const UartPins& p) { return p.tx.valid(); }

/// Two engines on one transport must not name the same STREAM: a stream
/// moves data one way and has one FIFO, and pointing both directions at
/// it would have each re-programming the other's block. Generic over any
/// engine that says `present`, `controller` and `stream` - `if constexpr`
/// keeps those from being looked up on an absent engine, which is what
/// lets NoDmaEngine stay a two-line tag.
template <typename Tx, typename Rx>
constexpr bool uart_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::controller != Rx::controller || Tx::stream != Rx::stream;
    } else {
        return true;
    }
}

/// Whether an engine sits on a cell of the request mapping that really
/// carries instance `n`'s transmit (or receive) request. True for an
/// absent engine - there is nothing to place - and FALSE on a part class
/// whose request table was not read, where the reserve refuses rather
/// than guesses (stm32f4/device_tables.hpp).
template <uint8_t n, typename E, bool transmit>
constexpr bool uart_engine_placed() {
    if constexpr (!E::present) {
        return true;
    } else {
        return usart_dma_placement_valid(n, transmit, E::controller, E::stream, E::channel);
    }
}

/// What a Uart may be told beyond its rings and engines, as one trailing
/// template parameter. The defaults are the console's personality and
/// compile to exactly the code they did before this struct existed.
struct UartOptions {
    /// OVER8: eight samples a bit, twice the reachable rate.
    bool over8 = false;
    /// ONEBIT: one sample a bit, no noise detection, more clock tolerance.
    bool one_bit = false;
    /// Single-wire half duplex (30.3.10): one pad, the TX pad, as an
    /// alternate-function OPEN DRAIN with a pull-up; the RX pad is left
    /// alone.
    bool half_duplex = false;
    /// The flow-control pair, each on the pad the datasheet gives it.
    bool rts = false;
    bool cts = false;
    PinSel rts_pin{};
    PinSel cts_pin{};
    /// The output driver's slew class on the TX pad: low (about 2 MHz
    /// class) serves the console rates; a multi-megabaud link asks for
    /// more.
    PinSpeed tx_speed = PinSpeed::low;
};

/**
 * Uart<n, pins, rx_size, tx_size, TxEngine, RxEngine, opts>: the
 * interrupt-driven byte transport.
 *
 *   constexpr brio::UartPins console_pins{
 *       .tx = {'A', 2, brio::PinFunction::af7},   // USART2_TX (DS10693 table 11)
 *       .rx = {'A', 3, brio::PinFunction::af7},   // USART2_RX
 *   };
 *   using Serial = brio::Uart<2, console_pins>;
 *   extern "C" void USART2_IRQHandler() {
 *       if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
 *   }
 *   Serial::init(clock, 115200);
 *
 * The shared Uart surface - same verbs, same return contracts:
 * init/rebase/set_baud/isr/write_byte/read_byte/write/write_bulk/
 * read_bulk, the error counters, release().
 */
template <uint8_t instance, UartPins pins, uint32_t rx_size = 64, uint32_t tx_size = 256,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine,
          UartOptions opts = UartOptions{}>
class Uart {
    using S = Usart<instance>;

    static_assert(opts.half_duplex ? uart_pins_valid_single(pins) : uart_pins_valid(pins),
                  "brio Uart: the two pads must be real pins of present ports, and TX "
                  "and RX cannot be the same pad (in half duplex only TX is claimed)");
    // The slots must name a COMPLETE type: naming it in a sizeof
    // instantiates the engine here, at the template argument the
    // application typed, so an engine's own static_asserts are reported
    // against the line that named it.
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from stm32f4/dma.hpp, or NoDmaEngine (the default)");
    static_assert(uart_engines_distinct<TxEngine, RxEngine>(),
                  "brio Uart: the transmit and receive engines must use DIFFERENT DMA "
                  "streams - a stream carries one direction and has one FIFO");
    static_assert(uart_engine_placed<instance, TxEngine, true>(),
                  "brio Uart: the transmit engine's (controller, stream, channel) is not "
                  "a cell this instance's transmit request is wired to - RM0090 tables 43 "
                  "and 44 and their RM0390 / RM0383 twins, keyed per part class in "
                  "stm32f4/device_tables.hpp (a part class whose manual was not read has "
                  "no table, and an engine is refused there rather than guessed)");
    static_assert(uart_engine_placed<instance, RxEngine, false>(),
                  "brio Uart: the receive engine's (controller, stream, channel) is not a "
                  "cell this instance's receive request is wired to - see the transmit "
                  "engine's message");
    static_assert(!opts.rts || opts.rts_pin.valid(), "brio Uart: RTS flow control needs the RTS pad");
    static_assert(!opts.cts || opts.cts_pin.valid(), "brio Uart: CTS flow control needs the CTS pad");
    static_assert(!(opts.rts || opts.cts) || S::is_full,
                  "brio Uart: hardware flow control is a FULL instance's (USART1, 2, 3, 6 - "
                  "RM0090 table 148); UART4/5/7/8 have no CTS/RTS");

    using TxPin = Pin<pins.tx.port, pins.tx.pin>;
    using RxPin = Pin<pins.rx.port, pins.rx.pin>;

    static inline Ring<uint8_t, rx_size, Stm32f4Platform<>> m_rx{};
    static inline Ring<uint8_t, tx_size, Stm32f4Platform<>> m_tx{};

    static inline volatile uint8_t m_rx_overruns = 0;   // RX ring full, byte lost
    static inline volatile uint8_t m_frame_errors = 0;  // FE: byte dropped
    static inline volatile uint8_t m_parity_errors = 0; // PE: byte dropped
    static inline volatile uint8_t m_noise_errors = 0;  // NE: byte kept, line suspect
    static inline volatile uint8_t m_hw_overruns = 0;   // ORE: a byte lost in silicon
    static inline volatile uint8_t m_dma_faults = 0;    // blocks a dead stream lost
    static inline uint32_t m_baud = 0;                  // for rebase()

public:
    constexpr Uart() = default;

    using Resource = S;

    /// Whether this instantiation carries an engine at all. Every engine
    /// branch below is behind one of these, so an engineless image has
    /// no DMA code in it.
    static constexpr bool has_tx_engine = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;
    static constexpr UartOptions options = opts;

    /// The rate the baud divisor really divides, for the app's clock:
    /// the instance's APB clock. A compile-time constant with a static
    /// clock.
    template <typename Clock>
    static constexpr uint32_t kernel_hz() {
        return apb_hz(Clock{}, S::on_apb2);
    }

    /// Bring the instance up: bus clock, frame, baud, the options, pads,
    /// the receive interrupt and its NVIC line.
    ///
    /// Call AFTER the main clock is set up and before interrupts are
    /// enabled globally; `clock` is the app's brio::Clock tag, and the
    /// divisor comes from THE INSTANCE'S APB CLOCK - never from a second
    /// statement of the rate, and never from SYSCLK by assumption. False
    /// when the rate cannot be produced at that clock: the caller then
    /// knows the transport is NOT up, rather than printing into a ring
    /// nothing will drain.
    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, const UartFormat& format = {}) {
        static_assert(clock_follows<Clock, Uart>(),
                      "this Uart is initialized with a DynamicClock that does not "
                      "list it among its Users: it would keep the old baud after "
                      "a clock change");
        if (format.bits == UartBits::nine) {
            return false;   // the rings carry bytes; the resource's word verbs speak nine
        }

        const uint32_t fck = apb_hz(clock, S::on_apb2);
        const std::optional<uint16_t> reg = divisor_for(fck, baud);
        if (!reg) {
            return false;
        }

        Nvic::disable(S::irq);
        m_rx.clear();
        m_tx.clear();
        clear_errors();
        m_baud = baud;

        S::bus_clock(true);
        S::reset();
        if constexpr (opts.over8) {
            (void)S::oversampling8(true);
        }
        if (!S::configure(format, *reg)) {
            return false;
        }
        if constexpr (opts.one_bit) {
            S::one_bit_sampling(true);
        }
        if constexpr (opts.half_duplex) {
            (void)S::half_duplex(true);
        }
        if constexpr (opts.rts || opts.cts) {
            (void)S::flow_control(opts.rts, opts.cts);
        }

        // Pads BEFORE the enable: TE's idle frame must land on a pad the
        // peripheral already owns. The RX pad gets a pull-up so an
        // unconnected line reads idle rather than noise.
        if constexpr (opts.half_duplex) {
            TxPin::function(pins.tx.function, {.pull = PinPull::up, .open_drain = true, .speed = opts.tx_speed});
        } else {
            TxPin::function(pins.tx.function, {.speed = opts.tx_speed});
            RxPin::function(pins.rx.function, {.pull = PinPull::up});
        }
        if constexpr (opts.rts) {
            Pin<opts.rts_pin.port, opts.rts_pin.pin>::function(opts.rts_pin.function);
        }
        if constexpr (opts.cts) {
            Pin<opts.cts_pin.port, opts.cts_pin.pin>::function(opts.cts_pin.function, {.pull = PinPull::up});
        }

        // CR3's two request bits before the enable, so the peripheral is
        // asking for a stream from its very first byte.
        if constexpr (has_tx_engine) {
            S::dma_transmit(true);
        }
        if constexpr (has_rx_engine) {
            S::dma_receive(true);
        }

        S::transmitter(true);
        S::receiver(true);
        S::enable(true);
        S::clear_by_read();
        if constexpr (has_rx_engine) {
            // NOT S::rxne_interrupt(true): the stream consumes RXNE, and a
            // handler that also read DR would race it for the byte.
            RxEngine::arm(S::data_address());
            rearm_rx();
        } else {
            S::rxne_interrupt(true);
        }
        if constexpr (has_tx_engine) {
            TxEngine::arm(S::data_address());
        }
        // TXE is armed on demand by write_byte() when there is no engine.

        Nvic::enable(S::irq);
        return true;
    }

    /// The ISR body of WHICHEVER DMA stream this transport owns - call it
    /// from the vector of each engine's stream (one vector per stream on
    /// this family, shared with nothing):
    ///
    ///     extern "C" void DMA2_Stream7_IRQHandler() { (void)Serial::dma_isr(); }
    ///
    /// On the transmit stream a completion means the block has left the
    /// ring, so exactly that many bytes are released and the next
    /// contiguous run started. On the receive stream nothing is published
    /// here - only harvest() knows how much of the run the consumer has
    /// been told about, and the pacing of that is the owner's.
    ///
    /// Returns true when something belonging to this transport was served.
    [[gnu::always_inline]] static bool dma_isr() {
        bool mine = false;
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service();
            if ((f & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                m_dma_faults = m_dma_faults + 1;
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
                m_dma_faults = m_dma_faults + 1;
                mine = true;
            } else if ((f & RxEngine::flag_complete) != 0u) {
                // The run filled up. harvest() publishes and re-arms.
                mine = true;
            }
        }
        return mine;
    }

    /**
     * Ask the receive engine what has arrived, and publish it.
     *
     * WHY THIS IS A VERB AND NOT AN INTERRUPT. A receive block completes
     * only when the buffer fills, which on an idle line may be never, so
     * there is no event to wait for. WHOEVER OWNS THE PORT DECIDES HOW
     * OFTEN TO ASK and pays the latency it chose; a kernel TimeEvent
     * every few ticks is the shape brio expects. The asking itself is one
     * SxNDTR read.
     *
     * THE ERRORS ARE READ AT HARVEST GRANULARITY, and clearing one costs
     * a byte: on this block ORE, FE, NE and PE go away only when SR is
     * read and then DR (30.6.1), and DR is what the stream reads. So the
     * clearing sequence runs only when a flag really stands, and the byte
     * it takes is counted as the loss it is.
     *
     * Returns true when the receive ring went from empty to non-empty -
     * the same edge contract isr() has, so the same kernel glue works.
     * False, and free, without an engine.
     */
    static bool harvest() {
        if constexpr (!has_rx_engine) {
            return false;
        } else {
            const uint32_t errors = S::status() & UsartFlag::receive_errors;
            if (errors != 0u) {
                if ((errors & UsartFlag::ore) != 0u) {
                    m_hw_overruns = m_hw_overruns + 1;
                }
                if ((errors & UsartFlag::fe) != 0u) {
                    m_frame_errors = m_frame_errors + 1;
                }
                if ((errors & UsartFlag::pe) != 0u) {
                    m_parity_errors = m_parity_errors + 1;
                }
                if ((errors & UsartFlag::ne) != 0u) {
                    m_noise_errors = m_noise_errors + 1;
                }
                S::clear_by_read();
            }

            const bool was_empty = m_rx.empty();
            const uint16_t fresh = RxEngine::take();
            if (fresh != 0u) {
                m_rx.publish(static_cast<typename decltype(m_rx)::index_t>(fresh));
            }
            // THE SILICON IS ASKED FIRST AND THE ARITHMETIC SECOND: a
            // stream that is not running gets a new run whatever the count
            // says. The opposite rule - trust the count and leave a stopped
            // stream alone - leaves a receive stream dead.
            if (RxEngine::idle() || RxEngine::full() || RxEngine::capacity() == 0u) {
                rearm_rx();
            }
            return was_empty && !m_rx.empty();
        }
    }

    /// DMA blocks this transport threw away because the controller had
    /// stopped running them (a transfer error clears EN in hardware,
    /// RM0090 10.3.18). Always 0, and free, without an engine.
    static uint8_t dma_faults() {
        if constexpr (has_tx_engine || has_rx_engine) {
            return m_dma_faults;
        } else {
            return 0;
        }
    }

    /// The core clock changed (DynamicClock fan-out): keep the same bit
    /// rate at the new APB rate. `hz` is SYSCLK; the APB divider in force
    /// is read back from the RCC. Drains what is in flight first. Main
    /// context only.
    static void rebase(uint32_t hz) {
        constexpr uint32_t ring_drain_spins = 8'000'000u;
        constexpr uint32_t frame_spins = 200'000u;
        uint32_t spins = ring_drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = frame_spins;
        while ((S::status() & UsartFlag::tc) == 0u && spins-- != 0u) {
        }
        const uint32_t fck = hz / (S::on_apb2 ? Rcc::apb2_divider() : Rcc::apb1_divider());
        const std::optional<uint16_t> reg = divisor_for(fck, m_baud);
        if (!reg) {
            return;
        }
        S::regs().BRR = *reg;
    }

    /// Move the LINK to a different bit rate, the clock staying put -
    /// the mirror of rebase(). `hz` is SYSCLK, as rebase() takes it.
    /// False, and nothing written, when the new rate is unreachable.
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const uint32_t fck = hz / (S::on_apb2 ? Rcc::apb2_divider() : Rcc::apb1_divider());
        const std::optional<uint16_t> reg = divisor_for(fck, baud);
        if (!reg) {
            return false;
        }
        constexpr uint32_t ring_drain_spins = 8'000'000u;
        constexpr uint32_t frame_spins = 200'000u;
        uint32_t spins = ring_drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = frame_spins;
        while ((S::status() & UsartFlag::tc) == 0u && spins-- != 0u) {
        }
        S::regs().BRR = *reg;
        m_baud = baud;
        return true;
    }

    /// The divisor `fck` and `baud` ask for, in the options' oversampling.
    static constexpr std::optional<uint16_t> divisor_for(uint32_t fck, uint32_t baud) {
        if constexpr (opts.over8) {
            return usart_brr_over8(fck, baud);
        } else {
            return usart_brr(fck, baud);
        }
    }
    static constexpr uint32_t min_hz_for(uint32_t baud) {
        return opts.over8 ? usart_min_hz_over8(baud) : usart_min_hz(baud);
    }
    /// Whether `baud` is reachable from an APB clock `fck`.
    static constexpr bool can_baud(uint32_t fck, uint32_t baud) { return divisor_for(fck, baud).has_value(); }
    /// The baud the port is ACTUALLY running at, from the divisor in the
    /// register at APB clock `fck` - what a program reports instead of
    /// what it asked for.
    static uint32_t actual_baud(uint32_t fck) {
        return opts.over8 ? usart_actual_baud_over8(fck, S::brr()) : usart_actual_baud(fck, S::brr());
    }

    /// The instance's ONE interrupt body - call from its vector.
    ///
    /// Returns true when the RX ring transitioned empty -> non-empty: the
    /// edge signal for kernel glue ("post RxActivity on true"). Every
    /// empty->non-empty transition reports true and the consumer only
    /// empties the ring by draining it, so no wakeup is ever lost.
    [[gnu::always_inline]] static bool isr() {
        USART_TypeDef& r = S::regs();
        const uint32_t st = r.SR;   // ONE read; the DR read below completes the clears
        bool edge = false;

        // With a receive engine the stream owns DR and RXNEIE is never
        // armed, so this branch must not exist: a handler that read DR
        // would take a byte out of the stream's hands.
        if constexpr (has_rx_engine) {
            (void)edge;
        } else if ((st & UsartFlag::rxne) != 0u || (st & UsartFlag::ore) != 0u) {
            // The DR read clears RXNE and, after the SR read above, ORE,
            // NE, FE, PE and IDLE (30.6.1). An overrun without RXNE is
            // the case where the byte was already taken but ORE stands:
            // the read clears it, the byte is noise.
            const uint8_t b = static_cast<uint8_t>(r.DR);
            const uint32_t err = st & UsartFlag::receive_errors;
            if (err != 0u) {
                if ((err & UsartFlag::fe) != 0u) {
                    m_frame_errors = m_frame_errors + 1;
                }
                if ((err & UsartFlag::pe) != 0u) {
                    m_parity_errors = m_parity_errors + 1;
                }
                if ((err & UsartFlag::ne) != 0u) {
                    m_noise_errors = m_noise_errors + 1;
                }
                if ((err & UsartFlag::ore) != 0u) {
                    m_hw_overruns = m_hw_overruns + 1;
                }
            }
            if ((st & UsartFlag::rxne) != 0u && (err & (UsartFlag::fe | UsartFlag::pe)) == 0u) {
                const bool was_empty = m_rx.empty();
                if (m_rx.push(b)) {
                    edge = was_empty;
                } else {
                    m_rx_overruns = m_rx_overruns + 1;
                }
            }
        }

        if ((st & UsartFlag::txe) != 0u && (r.CR1 & USART_CR1_TXEIE) != 0u) {
            const auto v = m_tx.pop();
            if (v) {
                r.DR = *v;
            } else {
                r.CR1 &= ~USART_CR1_TXEIE;   // ring dry: TXE would re-fire for ever
            }
        }
        return edge;
    }

    /**
     * Queue one byte; false when the ring is full (print() retries).
     *
     * WITH AN ENGINE, A REFUSED BYTE STILL NUDGES. print() answers a
     * false by trying again for ever, so a path that can leave the
     * transport unpoked stops the program - and the ring is full
     * precisely when nothing is draining it. The plain transport does not
     * need it (a push that filled the ring already armed TXE, and TXE is
     * a condition that cannot be missed), which is also why the
     * engineless image is byte-identical to the one built before these
     * slots were filled.
     */
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
            S::txe_interrupt(true);
        }
        return true;
    }

    /// Fetch one received byte; false when nothing is pending.
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

    /// Queue a run of bytes through the ring's contiguous span and arm
    /// the transmitter ONCE. Returns the number queued.
    static uint32_t write_bulk(std::span<const uint8_t> src) {
        uint32_t queued = 0;
        while (queued < src.size()) {
            auto dst = m_tx.write_span();
            if (dst.empty()) {
                break;
            }
            const uint32_t chunk = dst.size() < src.size() - queued
                                       ? static_cast<uint32_t>(dst.size())
                                       : static_cast<uint32_t>(src.size() - queued);
            for (uint32_t i = 0; i < chunk; ++i) {
                dst[i] = src[queued + i];
            }
            m_tx.publish(static_cast<typename decltype(m_tx)::index_t>(chunk));
            queued += chunk;
        }
        // ONE nudge for the whole run - the entire point of the verb.
        if constexpr (has_tx_engine) {
            pump_tx();
        } else if (queued != 0u) {
            S::txe_interrupt(true);
        }
        return queued;
    }

    /// Drain received bytes into `dst`; returns the number copied.
    static uint32_t read_bulk(std::span<uint8_t> dst) {
        uint32_t got = 0;
        while (got < dst.size()) {
            auto src = m_rx.read_span();
            if (src.empty()) {
                break;
            }
            const uint32_t chunk = src.size() < dst.size() - got
                                       ? static_cast<uint32_t>(src.size())
                                       : static_cast<uint32_t>(dst.size() - got);
            for (uint32_t i = 0; i < chunk; ++i) {
                dst[got + i] = src[i];
            }
            m_rx.consume(static_cast<typename decltype(m_rx)::index_t>(chunk));
            got += chunk;
        }
        return got;
    }

    static auto rx_pending() { return m_rx.count(); }
    static bool tx_idle() { return m_tx.empty(); }

    static uint8_t rx_overruns() { return m_rx_overruns; }
    static uint8_t frame_errors() { return m_frame_errors; }
    static uint8_t parity_errors() { return m_parity_errors; }
    static uint8_t noise_errors() { return m_noise_errors; }
    static uint8_t hw_overruns() { return m_hw_overruns; }

    static void clear_errors() {
        m_rx_overruns = 0;
        m_frame_errors = 0;
        m_parity_errors = 0;
        m_noise_errors = 0;
        m_hw_overruns = 0;
        if constexpr (has_tx_engine || has_rx_engine) {
            m_dma_faults = 0;
        }
    }

    /// Stop the port and park its pads: interrupts off, UE clear, the
    /// bus clock closed, the pins released. The engines' streams are
    /// stopped first, because 10.3.17's warning is explicit - switch the
    /// stream off and wait for EN to read 0 BEFORE the peripheral it
    /// serves.
    static void release() {
        Nvic::disable(S::irq);
        if constexpr (has_tx_engine) {
            TxEngine::stop();
        }
        if constexpr (has_rx_engine) {
            RxEngine::stop();
        }
        S::rxne_interrupt(false);
        S::txe_interrupt(false);
        S::enable(false);
        S::bus_clock(false);
        TxPin::release();
        if constexpr (!opts.half_duplex) {
            RxPin::release();
        }
    }

private:
    /**
     * Hand the transmit engine the ring's next contiguous run, if it is
     * free to take one.
     *
     * NOTHING KICKS THE FIRST BEAT, AND THAT IS THIS CONTROLLER'S OWN
     * FACT. 10.3.2's handshake is level-driven: the stream is enabled, it
     * sees TXE asserted, it writes DR. Measured rather than assumed,
     * because a wrong answer is a transmitter that never starts.
     */
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            typename Stm32f4Platform<>::CriticalSection cs;
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

    /// Point the receive engine at the ring's next free run. A ring with
    /// no room at all is a byte lost before it arrives, and it is counted
    /// as the software overrun it is.
    static void rearm_rx() {
        if constexpr (has_rx_engine) {
            const auto room = m_rx.write_span();
            if (room.empty()) {
                m_rx_overruns = m_rx_overruns + 1;
                return;
            }
            (void)RxEngine::start(room.data(), static_cast<uint16_t>(room.size()));
        }
    }
};

} // namespace brio
