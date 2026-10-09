/*
 * usart.hpp
 *
 * The USART (RM0444 ch. 33) in the two strata every brio serial driver
 * has (docs/design/serial.md):
 *
 *  Usart<n>              the RESOURCE: which instance, where its registers
 *                        are, its bus clock, its kernel-clock multiplexer
 *                        where it has one, its NVIC line, its EXTI wake
 *                        line, the enable discipline, the whole register
 *                        description of chapter 33 and the flag surface
 *                        in BOTH register views;
 *  UartTask<...>         the TASK: the asynchronous byte transport with
 *                        ring buffers and one ISR body - every console's
 *                        personality, a ByteTransport for print() and
 *                        SerialPort. `Uart<n, pins, ...>` names it over a
 *                        Usart<n> and stm32g0/lpuart.hpp's `LpUart<n,
 *                        pins, ...>` names the SAME task over an
 *                        Lpuart<n>: one implementation, two peripherals,
 *                        because chapter 34 is chapter 33 with a
 *                        different baud generator. Its public surface
 *                        is every target's Uart surface, which is what
 *                        lets util/serial_port.hpp and print() compile
 *                        here untouched.
 *
 * SCOPE. The chapter, whole, bar what is declined with a reason in
 * docs/stm32g0/usart.md: every frame format, both oversamplings, the
 * prescaler, all four kernel clocks, the FIFOs and their thresholds,
 * single-wire half-duplex, LIN, IrDA, smartcard, synchronous master,
 * hardware flow control and driver enable, mute mode, character match,
 * the receiver time-out and Modbus, auto-baud, the swap/invert options,
 * the request register and the wake from Stop. DMA rides the task's two
 * OPTIONAL engine slots (stm32g0/dma.hpp, NoDmaEngine).
 *
 * THE INSTANCE SPLIT IS THE FIRST FACT OF THE CHAPTER (tables 183/184):
 * an instance is FULL, BASIC or LP, and a BASIC one has no FIFO, no
 * prescaler, no kernel-clock multiplexer, no wake from Stop, no
 * synchronous mode, no smartcard, no IrDA, no LIN, no auto-baud, no
 * receiver time-out and no Modbus. `usart_is_full(n)` in the reserve is
 * the compile-time answer (a stated table: the device header expresses
 * the split as POINTER-COMPARISON macros, which are not constant
 * expressions) and the `has_*()` verbs below are the RUNTIME reads of
 * those very macros - so a suite can compare the table, the header and
 * the silicon against each other, which is what the bench does. Every
 * feature verb a BASIC instance lacks REFUSES on one (false, nothing
 * written).
 *
 * Facts that shape the code (RM0444 33.5, 33.8; ES0548 rev Z):
 *  - BRR, CR1's frame fields, CR2, CR3's mode fields, GTPR and PRESC are
 *    written only with UE = 0 (33.8.x "can only be written when the USART
 *    is disabled" - for CR3 bit by bit, 33.8.4, and DMAT and DMAR carry
 *    no such clause); the task disables around every configuration, including
 *    rebase(), and every resource verb below that touches such a field
 *    REFUSES while the instance is enabled rather than storing into a
 *    register the silicon ignores;
 *  - the baud generator (33.5.7): USARTDIV = usart_ker_ck_pres / baud
 *    with OVER8 = 0 and 2 x that with OVER8 = 1, USARTDIV >= 16 in both
 *    - and under OVER8 the register is NOT the divisor: BRR[15:4] =
 *    USARTDIV[15:4], BRR[2:0] = USARTDIV[3:0] >> 1, BRR[3] = 0. The
 *    kernel clock is what the divisor divides, and it is
 *    usart_ker_ck AFTER the PRESC prescaler, not PCLK by assumption;
 *  - TXE is a CONDITION (transmit data register empty), so its interrupt
 *    is armed only while the ring holds something and disarmed from the
 *    handler when it runs dry - and so are TXFNF and TXFT, the FIFO
 *    view's two, of which the task rides TXFT by default: an instance
 *    with the 8-deep FIFOs (table 184) runs the task in FIFO mode
 *    unless the program says otherwise, one interrupt refilling the
 *    whole transmit FIFO and the receiver nine characters deep;
 *  - ORE raises the interrupt whenever RXNEIE is set (33.8.9), and it is
 *    cleared ONLY through ICR.ORECF - a handler that reads RDR and
 *    leaves ORE standing re-enters for ever. Every error flag has its
 *    ICR twin and the handler clears what it counts;
 *  - RDR holds the LAST GOOD byte when ORE is set (the lost one is the
 *    next); FE/NE/PE flags belong to the byte in RDR, so a framed or
 *    parity-failed byte is dropped precisely - and in FIFO mode 33.5.4
 *    stores those three flags WITH EACH ENTRY, so a draining handler
 *    reads ISR BEFORE each RDR or the attribution slides by one;
 *  - TE set sends an idle frame first (33.5.5), which is why the pads
 *    are handed to the peripheral BEFORE UE/TE are raised;
 *  - single-wire half-duplex (33.5.15) internally connects TX and RX,
 *    stops using the RX pin and RELEASES the TX pin whenever it is not
 *    transmitting - so the pad must be alternate-function OPEN DRAIN
 *    with a pull-up, and the instance then hears every byte it sends.
 *    That is this family's loop-back (there is no LBME here) and it is
 *    the instrument the whole bench suite is built on;
 *  - ES0548 2.11.1: a glitch to zero shorter than half a bit inside the
 *    SECOND half of a stop bit corrupts the received byte, no
 *    workaround - the noise flag NE is what such a line shows first, and
 *    the task counts it separately for that reason;
 *  - ES0548 2.11.2 is a documentation erratum about exactly the
 *    per-instance prescaler split above; usart_is_full() carries it;
 *  - the vector is SHARED on this family (device_tables.hpp): the app's
 *    handler for USART2_LPUART2_IRQHandler calls this instance's isr()
 *    and, if it uses LPUART2, that one's too.
 */

#pragma once

#include <stdint.h>
#include <cstring>
#include <optional>
#include <span>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/dma_engine.hpp"
#include "stm32g0/exti.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

// ---- frame vocabulary ---------------------------------------------------------

/// CR1.M1:M0 - the word length INCLUDING the parity bit (33.5.5: with
/// parity on, one of the data bits becomes the parity bit).
enum class UartBits : uint8_t {
    seven = 7,
    eight = 8,
    nine = 9,
};

enum class UartParity : uint8_t { none, even, odd };

/// The frame the two ends agree on. Defaults to 8N1.
struct UartFormat {
    UartBits bits = UartBits::eight;
    UartParity parity = UartParity::none;
    uint8_t stop_bits = 1;   ///< 1 or 2 (the 0.5/1.5 codes are smartcard's)
};

constexpr bool uart_format_valid(const UartFormat& f) {
    return (f.stop_bits == 1 || f.stop_bits == 2);
}

/// CR2.STOP, all four codes - 0.5 and 1.5 belong to smartcard mode
/// (33.5.17) and cannot be reached through UartFormat, which speaks
/// the shared asynchronous vocabulary alone.
enum class UartStop : uint8_t { one = 0, half = 1, two = 2, one_and_half = 3 };

/// The two pads of a link, each with the AF the datasheet gives the
/// signal on that pad (DS13560 tables 13..24). No header symbol can
/// check the AF number: the datasheet is the authority, the bench the
/// proof.
struct UartPins {
    PinSel tx;
    PinSel rx;
};

constexpr bool uart_pins_valid(const UartPins& p) {
    return p.tx.valid() && p.rx.valid() &&
           !(p.tx.port == p.rx.port && p.tx.pin == p.rx.pin);
}

/// A single-wire link (33.5.15 half-duplex, and smartcard's own single
/// wire): only the TX pad exists, and it is open drain with a pull-up.
constexpr bool uart_pins_valid_single(const UartPins& p) { return p.tx.valid(); }

// ---- the prescaler and the kernel clock (33.8.14, 5.4.21) ----------------------

/// USART_PRESC.PRESCALER, the twelve implemented codes of 33.8.14. The
/// Reserved ones are not spelled, and the chapter's own note says the
/// silicon FORCES a Reserved code to 1011 (divide by 256) rather than
/// ignoring it - which is why prescaler() refuses instead of trusting.
enum class UsartPrescaler : uint8_t {
    div1 = 0, div2 = 1, div4 = 2, div6 = 3, div8 = 4, div10 = 5,
    div12 = 6, div16 = 7, div32 = 8, div64 = 9, div128 = 10, div256 = 11,
};

constexpr uint32_t usart_prescaler_divisor(UsartPrescaler p) {
    switch (p) {
        case UsartPrescaler::div1: return 1;
        case UsartPrescaler::div2: return 2;
        case UsartPrescaler::div4: return 4;
        case UsartPrescaler::div6: return 6;
        case UsartPrescaler::div8: return 8;
        case UsartPrescaler::div10: return 10;
        case UsartPrescaler::div12: return 12;
        case UsartPrescaler::div16: return 16;
        case UsartPrescaler::div32: return 32;
        case UsartPrescaler::div64: return 64;
        case UsartPrescaler::div128: return 128;
        default: return 256;
    }
}

constexpr bool usart_prescaler_valid(UsartPrescaler p) {
    return static_cast<uint8_t>(p) <= 11u;
}

/// The kernel clock an instance with a multiplexer may take (CCIPR
/// USARTnSEL / LPUARTnSEL codes, 5.4.21 - the same four codes and the
/// same order for every one of them).
enum class UsartClock : uint8_t { pclk = 0, sysclk = 1, hsi16 = 2, lse = 3 };

/// usart_ker_ck_pres: what the baud divisor really divides - the kernel
/// clock AFTER the prescaler (33.5.7's own name for it).
constexpr uint32_t usart_kernel_hz(uint32_t ker_hz, UsartPrescaler p) {
    return ker_hz / usart_prescaler_divisor(p);
}

/// The rate a kernel-clock CODE is worth, given the application's clock
/// tag. HSI16 is 16 MHz by construction and LSE is the 32768 Hz crystal;
/// PCLK and SYSCLK come from the clock type, which is the one truth
/// about them (there is no F_CPU in this build).
template <typename Clock>
constexpr uint32_t usart_kernel_clock_hz(UsartClock c) {
    switch (c) {
        case UsartClock::pclk: return Clock::pclk_hz;
        case UsartClock::sysclk: return Clock::hz;
        case UsartClock::hsi16: return 16'000'000u;
        default: return 32768u;
    }
}

// ---- baud arithmetic (33.5.7) --------------------------------------------------

/// BRR for `baud` at kernel clock `hz` (already through the prescaler),
/// with OVER8 = 0: USARTDIV = hz / baud and BRR = USARTDIV, USARTDIV in
/// 16..65535. Rounded to nearest; nothing when the rate is unreachable.
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

/// The same with OVER8 = 1 - and the register is NOT the divisor:
/// USARTDIV = 2 x hz / baud (still >= 16), BRR[15:4] = USARTDIV[15:4],
/// BRR[2:0] = USARTDIV[3:0] >> 1, BRR[3] kept clear (33.5.7).
///
/// IT IS A SIBLING VERB AND NOT A DEFAULTED THIRD ARGUMENT, and the
/// reason is measured: giving usart_brr() a `bool over8 = false` moves
/// an image by forty bytes although the folded code for `false` is
/// identical. Byte identity outranks API economy.
constexpr std::optional<uint16_t> usart_brr_over8(uint32_t hz, uint32_t baud) {
    if (hz == 0u || baud == 0u) {
        return std::nullopt;
    }
    const uint32_t div = ((hz << 1) + baud / 2u) / baud;
    if (div < 16u || div > 0xFFFFu) {
        return std::nullopt;
    }
    return static_cast<uint16_t>((div & 0xFFF0u) | ((div & 0x000Fu) >> 1));
}

/// What the generator really produces for a BRR value.
constexpr uint32_t usart_actual_baud(uint32_t hz, uint16_t brr) {
    return brr == 0u ? 0u : hz / brr;
}

/// The same read back through the OVER8 encoding above.
constexpr uint32_t usart_actual_baud_over8(uint32_t hz, uint16_t brr) {
    const uint32_t div = (static_cast<uint32_t>(brr) & 0xFFF0u) |
                         ((static_cast<uint32_t>(brr) & 0x0007u) << 1);
    return div == 0u ? 0u : (2u * hz) / div;
}

/// The lowest kernel clock that still reaches `baud` (USARTDIV >= 16).
constexpr uint32_t usart_min_hz(uint32_t baud) { return baud * 16u; }
/// Half of it, which is what OVER8 buys.
constexpr uint32_t usart_min_hz_over8(uint32_t baud) { return baud * 8u; }

// ---- the rest of the chapter's vocabulary --------------------------------------

/// CR3.RXFTCFG / CR3.TXFTCFG (33.5.4, 33.8.4). `full_or_empty` is code
/// 101: RXFIFO full on the receive side, TXFIFO empty on the transmit
/// side. `none` is brio's own and not a code: it means "leave the
/// threshold at its reset value and do not arm its interrupt", which is
/// what a transport pacing itself on RXFNE/TXFNF wants.
enum class UartFifoThreshold : uint8_t {
    eighth = 0, quarter = 1, half = 2, three_quarters = 3,
    seven_eighths = 4, full_or_empty = 5,
    none = 0xFF,
};

constexpr bool uart_fifo_threshold_valid(UartFifoThreshold t) {
    return t == UartFifoThreshold::none || static_cast<uint8_t>(t) <= 5u;
}

/// CR3.WUS (33.8.4): which event raises WUF. Code 01 is Reserved and is
/// not spelled; `none` is brio's own for "do not arm the wake at all".
enum class UsartWakeSource : uint8_t {
    address_match = 0,
    start_bit = 2,
    receive_ready = 3,     ///< RXNE / RXFNE
    none = 0xFF,
};

/// USART_RQR (33.8.8): five write-only strobes.
enum class UsartRequest : uint8_t {
    auto_baud = USART_RQR_ABRRQ,
    send_break = USART_RQR_SBKRQ,
    mute = USART_RQR_MMRQ,
    flush_receive = USART_RQR_RXFRQ,
    flush_transmit = USART_RQR_TXFRQ,
};

/// CR2.ABRMODE (33.5.9): which character pattern the auto-baud unit
/// measures. `off` is brio's own.
enum class AutoBaudMode : uint8_t {
    start_bit = 0,      ///< any character starting with a 1
    falling_edges = 1,  ///< any character with a 10xx pattern
    frame_7f = 2,       ///< a 0x7F frame
    frame_55 = 3,       ///< a 0x55 frame
    off = 0xFF,
};

/// CR1.WAKE (33.5.10): how a muted receiver comes back.
enum class MuteWake : uint8_t { idle_line = 0, address_mark = 1 };

/// Mute mode: MME + WAKE + ADDM7 + CR2.ADD (33.5.10). `address` is the
/// 4- or 7-bit address the receiver answers to under address-mark
/// wake-up; under idle-line wake-up it is not consulted.
struct MuteConfig {
    MuteWake wake = MuteWake::idle_line;
    bool address_7bit = false;   ///< CR2.ADDM7: 7-bit address instead of 4-bit
    uint8_t address = 0;
};

/// CR3.DEM/DEP + CR1.DEAT/DEDT (33.5.20). THE UNITS ARE SAMPLE TIMES -
/// 1/16 of a bit with OVER8 = 0 and 1/8 with OVER8 = 1 - and not bit
/// times, which is the trap in that section's one sentence about them.
/// The DE signal comes out on the RTS pad.
struct DriverEnableConfig {
    uint8_t assertion = 0;      ///< DEAT, 0..31 sample times before the start bit
    uint8_t deassertion = 0;    ///< DEDT, 0..31 sample times after the last stop bit
    bool active_low = false;    ///< DEP
};

constexpr bool driver_enable_valid(const DriverEnableConfig& c) {
    return c.assertion <= 31u && c.deassertion <= 31u;
}

/// LIN mode (33.5.13): LINEN plus the break-detection length. The
/// chapter FORBIDS company here - STOP, CLKEN, SCEN, HDSEL and IREN must
/// all be clear - and 8-bit words are the only legal frame, so lin()
/// refuses anything else rather than writing a combination the silicon
/// does not define.
struct LinConfig {
    bool break_11bit = false;   ///< LBDL: 11-bit detection instead of 10
    bool break_interrupt = false;
};

/// Synchronous master (33.5.14): the CK output beside TX.
struct SyncConfig {
    bool clock_idle_high = false;   ///< CPOL
    bool sample_second_edge = false; ///< CPHA
    bool last_bit_clock = false;    ///< LBCL: a clock pulse on the last data bit too
};

/// Smartcard (33.5.17): a single wire on the TX pad, 8 bits + parity,
/// 1.5 stop bits, a guard time counted in baud periods after the stop
/// bit, an optional CK output divided by 2 x PSC, automatic NACK and up
/// to seven auto-retries.
struct SmartcardConfig {
    bool nack = true;            ///< CR3.NACK: NACK a parity error (T = 0)
    uint8_t retries = 0;         ///< CR3.SCARCNT, 0..7
    uint8_t guard_time = 0;      ///< GTPR.GT, in baud periods
    uint8_t clock_prescaler = 0; ///< GTPR.PSC[4:0]; 0 = no CK output, else CK = ker/(2 x PSC)
    bool clock_output = false;   ///< CR2.CLKEN
    bool receive_half_stop = false; ///< STOP = 01 (0.5 stop bit) instead of 11 (1.5)
    uint8_t block_length = 0;    ///< RTOR.BLEN (T = 1 block mode)
};

constexpr bool smartcard_valid(const SmartcardConfig& c) {
    // 33.8.6: PSC[7:5] must be kept clear in smartcard mode, and 00000
    // is "Reserved - do not program this value" when a CK output is
    // wanted at all.
    return c.retries <= 7u && c.clock_prescaler <= 31u &&
           (!c.clock_output || c.clock_prescaler != 0u);
}

/// IrDA SIR ENDEC (33.5.18): IREN plus the low-power divisor. The
/// encoder/decoder DOES NOT WORK WITH PSC = 0 (the chapter says so in
/// as many words), the stop bits must be 1, and LINEN/CLKEN/SCEN/HDSEL
/// must be clear.
struct IrdaConfig {
    bool low_power = false;   ///< CR3.IRLP
    uint8_t prescaler = 1;    ///< GTPR.PSC[7:0], the low-power divisor; also the glitch filter
};

constexpr bool irda_valid(const IrdaConfig& c) { return c.prescaler != 0u; }

// ---- the resource -------------------------------------------------------------

/// USART_ISR bits, by BOTH names the register description gives them:
/// the non-FIFO spelling and, where the same bit changes meaning with
/// FIFOEN, the FIFO one beside it (the device header's own dual macros).
/// The four flags at the top - TXFE, RXFF, RXFT, TXFT - exist only in
/// the FIFO view.
struct UsartFlag {
    static constexpr uint32_t pe = USART_ISR_PE;
    static constexpr uint32_t fe = USART_ISR_FE;
    static constexpr uint32_t ne = USART_ISR_NE;
    static constexpr uint32_t ore = USART_ISR_ORE;
    static constexpr uint32_t idle = USART_ISR_IDLE;
    static constexpr uint32_t rxne = USART_ISR_RXNE_RXFNE;
    static constexpr uint32_t rxfne = USART_ISR_RXNE_RXFNE;
    static constexpr uint32_t tc = USART_ISR_TC;
    static constexpr uint32_t txe = USART_ISR_TXE_TXFNF;
    static constexpr uint32_t txfnf = USART_ISR_TXE_TXFNF;
    static constexpr uint32_t lbdf = USART_ISR_LBDF;
    static constexpr uint32_t ctsif = USART_ISR_CTSIF;
    static constexpr uint32_t cts = USART_ISR_CTS;
    static constexpr uint32_t rtof = USART_ISR_RTOF;
    static constexpr uint32_t eobf = USART_ISR_EOBF;
    static constexpr uint32_t udr = USART_ISR_UDR;
    static constexpr uint32_t abre = USART_ISR_ABRE;
    static constexpr uint32_t abrf = USART_ISR_ABRF;
    static constexpr uint32_t busy = USART_ISR_BUSY;
    static constexpr uint32_t cmf = USART_ISR_CMF;
    static constexpr uint32_t sbkf = USART_ISR_SBKF;
    static constexpr uint32_t rwu = USART_ISR_RWU;
    static constexpr uint32_t wuf = USART_ISR_WUF;
    static constexpr uint32_t teack = USART_ISR_TEACK;
    static constexpr uint32_t reack = USART_ISR_REACK;
    static constexpr uint32_t txfe = USART_ISR_TXFE;
    static constexpr uint32_t rxff = USART_ISR_RXFF;
    static constexpr uint32_t tcbgt = USART_ISR_TCBGT;
    static constexpr uint32_t rxft = USART_ISR_RXFT;
    static constexpr uint32_t txft = USART_ISR_TXFT;
    static constexpr uint32_t receive_errors = ore | fe | ne | pe;
};

/// USART_ICR: the write-1-to-clear twins. Every clearable ISR bit has
/// one and they sit at the SAME positions - which is why the handlers
/// below can write an ISR mask straight into ICR.
struct UsartClear {
    static constexpr uint32_t pe = USART_ICR_PECF;
    static constexpr uint32_t fe = USART_ICR_FECF;
    static constexpr uint32_t ne = USART_ICR_NECF;
    static constexpr uint32_t ore = USART_ICR_ORECF;
    static constexpr uint32_t idle = USART_ICR_IDLECF;
    static constexpr uint32_t txfe = USART_ICR_TXFECF;
    static constexpr uint32_t tc = USART_ICR_TCCF;
    static constexpr uint32_t tcbgt = USART_ICR_TCBGTCF;
    static constexpr uint32_t lbdf = USART_ICR_LBDCF;
    static constexpr uint32_t ctsif = USART_ICR_CTSCF;
    static constexpr uint32_t rtof = USART_ICR_RTOCF;
    static constexpr uint32_t eobf = USART_ICR_EOBCF;
    static constexpr uint32_t udr = USART_ICR_UDRCF;
    static constexpr uint32_t cmf = USART_ICR_CMCF;
    static constexpr uint32_t wuf = USART_ICR_WUCF;
    static constexpr uint32_t receive_errors = ore | fe | ne | pe;
    static constexpr uint32_t all = pe | fe | ne | ore | idle | txfe | tc | tcbgt |
                                    lbdf | ctsif | rtof | eobf | udr | cmf | wuf;
};

/// CR1/CR3 interrupt enables, gathered so a caller names one thing.
struct UsartInterrupt {
    static constexpr uint32_t idle = USART_CR1_IDLEIE;
    static constexpr uint32_t rxne = USART_CR1_RXNEIE_RXFNEIE;
    static constexpr uint32_t tc = USART_CR1_TCIE;
    static constexpr uint32_t txe = USART_CR1_TXEIE_TXFNFIE;
    static constexpr uint32_t parity = USART_CR1_PEIE;
    static constexpr uint32_t character_match = USART_CR1_CMIE;
    static constexpr uint32_t receiver_timeout = USART_CR1_RTOIE;
    static constexpr uint32_t end_of_block = USART_CR1_EOBIE;
    static constexpr uint32_t tx_fifo_empty = USART_CR1_TXFEIE;
    static constexpr uint32_t rx_fifo_full = USART_CR1_RXFFIE;
};

/**
 * Usart<n>: the instance. Every verb is one register fact; the ORDER
 * (configure disabled, pads before enable) is the task's.
 *
 * THE ENABLE RULE, everywhere: a field 33.8.x calls "can only be written
 * when the USART is disabled" is written by a verb that returns false
 * and STORES NOTHING while UE stands. The silicon's own behaviour is not
 * uniform enough to trust (elsewhere in this stratum a forbidden write
 * has been measured landing anyway), so the refusal is the driver's.
 */
template <uint8_t n>
struct Usart {
    static_assert(usart_present(n),
                  "brio Usart: this device has no such USART instance (the device header "
                  "declares no USARTn_BASE for it: 1..2 on the G031 class, 1..4 on the "
                  "G071 class, 1..6 on the G0B1 class)");

    Usart() = delete;

    static constexpr uint8_t index = n;
    static constexpr bool has_clock_select = usart_has_clock_select(n);

    /// Table 183's FULL/BASIC split, the compile-time half (the reserve
    /// states it: the header's own answer is a pointer comparison).
    static constexpr bool is_full = usart_is_full(n);
    /// The features that follow from it and have no header macro of
    /// their own: the prescaler (ES0548 2.11.2's subject), synchronous
    /// mode, the receiver time-out and Modbus.
    static constexpr bool has_prescaler = is_full;
    /// The FIFOs of table 184 - a FULL instance's here, and every
    /// LPUART's, which is why this is a flag of its own and not another
    /// reading of `is_full`.
    static constexpr bool has_fifo_mode = is_full;
    /// SYNCHRONOUS MODE IS *NOT* A FULL INSTANCE'S PRIVILEGE, and it is
    /// the one row of table 184 that breaks the FULL/BASIC reading:
    /// "Synchronous mode (Master/Slave) X X -" gives it to the FULL and
    /// the BASIC column and denies it to the LP one. 33.8.3's own note on
    /// CR2.CLKEN says the same thing the other way round ("if neither
    /// synchronous mode nor smartcard mode is supported, this bit is
    /// reserved"), and the device header's IS_USART_INSTANCE - the macro
    /// whose comment reads "USART Instances : Synchronous mode" - lists
    /// all six USARTs. So every USART of this family has a CK output and
    /// only the LPUARTs have none.
    static constexpr bool has_synchronous_mode = true;
    static constexpr bool has_receiver_timeout = is_full;
    /// LIN, on the other hand, IS a FULL instance's (table 184 and
    /// IS_UART_LIN_INSTANCE agree), and 33.8.3's note on CR2.LBDL says
    /// the bit is reserved where the mode is absent.
    static constexpr bool has_lin_mode = is_full;
    /// FIFO depth in characters (table 184): 8 on a FULL instance, and
    /// on a BASIC one the single data register, which is a depth of 1.
    static constexpr uint8_t fifo_depth = is_full ? 8u : 1u;
    /// The EXTI line the wake-up raises, 0xFF where there is no wake.
    static constexpr uint8_t exti_line = usart_exti_line(n);
    /// LPUART-ness, so a shared task can branch on the arithmetic.
    static constexpr bool is_lpuart = false;
    static constexpr bool has_oversampling8 = true;

    static USART_TypeDef& regs() { return *reinterpret_cast<USART_TypeDef*>(usart_base(n)); }
    static constexpr IRQn_Type irq() { return usart_irq(n); }

    /**
     * THE HEADER'S OWN ANSWER to the same question, at run time.
     *
     * IS_UART_FIFO_INSTANCE(x) and its nine siblings are pointer
     * comparisons - `((x) == USART1) || ...` - so they are perfectly
     * good expressions and perfectly useless as constant ones. Folded
     * against a constant `&regs()` they cost nothing, and they are the
     * SECOND opinion the bench compares the reserve's stated table and
     * the silicon against. Where the header has no macro - the
     * prescaler, synchronous mode, the receiver time-out - the constants
     * above are the only claim, and the bench asks the silicon directly.
     */
    static bool has_fifo() { return IS_UART_FIFO_INSTANCE(&regs()) != 0; }
    static bool has_autobaud() {
        return IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(&regs()) != 0;
    }
    static bool has_lin() { return IS_UART_LIN_INSTANCE(&regs()) != 0; }
    static bool has_irda() { return IS_IRDA_INSTANCE(&regs()) != 0; }
    static bool has_smartcard() { return IS_SMARTCARD_INSTANCE(&regs()) != 0; }
    static bool has_half_duplex() { return IS_UART_HALFDUPLEX_INSTANCE(&regs()) != 0; }
    static bool has_flow_control() { return IS_UART_HWFLOW_INSTANCE(&regs()) != 0; }
    static bool has_driver_enable() { return IS_UART_DRIVER_ENABLE_INSTANCE(&regs()) != 0; }
    static bool has_wake_from_stop() {
        return IS_UART_WAKEUP_FROMSTOP_INSTANCE(&regs()) != 0;
    }
    /// IS_USART_INSTANCE is the header's "USART Instances : Synchronous
    /// mode" macro - the second authority on has_synchronous_mode above,
    /// and the one the bench compares it with.
    static bool has_synchronous() { return IS_USART_INSTANCE(&regs()) != 0; }

    /// The APB clock of the block (RCC_APBENR1/2). Dead registers without it.
    static void bus_clock(bool on) {
        constexpr UsartBusClock bc = usart_bus_clock(n);
        if constexpr (bc.apb2) {
            Rcc::apb2_clock(bc.mask, on);
        } else {
            Rcc::apb1_clock(bc.mask, on);
        }
    }

    /// The kernel clock (CCIPR.USARTnSEL) where the instance has the
    /// multiplexer; an instance without one runs on PCLK and this verb
    /// answers whether the request is that one thing.
    static bool kernel_clock(UsartClock c) {
        if constexpr (has_clock_select) {
            Rcc::kernel_clock(usart_clock_select_pos(n), static_cast<uint8_t>(c));
            return true;
        } else {
            return c == UsartClock::pclk;
        }
    }
    /// What the multiplexer holds now. Always pclk where there is none.
    static UsartClock kernel_clock() {
        if constexpr (has_clock_select) {
            return static_cast<UsartClock>(Rcc::kernel_clock(usart_clock_select_pos(n)));
        } else {
            return UsartClock::pclk;
        }
    }

    static bool enabled() { return (regs().CR1 & USART_CR1_UE) != 0u; }
    static void enable(bool on) {
        regs().CR1 = on ? (regs().CR1 | USART_CR1_UE) : (regs().CR1 & ~USART_CR1_UE);
    }

    /// The whole frame + baud configuration, written with UE clear and
    /// the receiver/transmitter enables set for the moment UE rises.
    /// Refused (false, nothing written) while the instance is enabled.
    static bool configure(const UartFormat& f, uint16_t brr) {
        if (enabled() || !uart_format_valid(f)) {
            return false;
        }
        USART_TypeDef& r = regs();
        uint32_t cr1 = USART_CR1_TE | USART_CR1_RE;
        switch (f.bits) {
            case UartBits::seven: cr1 |= USART_CR1_M1; break;
            case UartBits::nine: cr1 |= USART_CR1_M0; break;
            default: break;
        }
        if (f.parity != UartParity::none) {
            cr1 |= USART_CR1_PCE;
            if (f.parity == UartParity::odd) {
                cr1 |= USART_CR1_PS;
            }
        }
        r.CR1 = cr1;                                          // OVER8 = 0, FIFOEN = 0
        r.CR2 = f.stop_bits == 2 ? USART_CR2_STOP_1 : 0u;     // 10 = 2 stop bits
        r.CR3 = 0;
        r.PRESC = 0;
        r.BRR = brr;
        return true;
    }

    /// CR2.STOP written on its own, for the two codes UartFormat cannot
    /// name (0.5 and 1.5 - smartcard's). Enable-protected.
    static bool stop_bits(UartStop s) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = (regs().CR2 & ~USART_CR2_STOP) |
                     (static_cast<uint32_t>(s) << USART_CR2_STOP_Pos);
        return true;
    }

    static uint16_t brr() { return static_cast<uint16_t>(regs().BRR); }
    static void set_brr(uint16_t v) { regs().BRR = v; }

    /// BRR for this peripheral's own arithmetic - what a shared task
    /// asks instead of knowing which of the two baud generators it is
    /// driving (stm32g0/lpuart.hpp answers the same question its way).
    [[gnu::always_inline]] static constexpr std::optional<uint16_t> brr_for(uint32_t ker_hz, uint32_t baud) {
        return usart_brr(ker_hz, baud);
    }
    [[gnu::always_inline]] static constexpr std::optional<uint16_t> brr_for_over8(uint32_t ker_hz,
                                                           uint32_t baud) {
        return usart_brr_over8(ker_hz, baud);
    }
    [[gnu::always_inline]] static constexpr uint32_t baud_for(uint32_t ker_hz, uint16_t reg) {
        return usart_actual_baud(ker_hz, reg);
    }
    [[gnu::always_inline]] static constexpr uint32_t baud_for_over8(uint32_t ker_hz, uint16_t reg) {
        return usart_actual_baud_over8(ker_hz, reg);
    }
    [[gnu::always_inline]] static constexpr uint32_t min_hz_for(uint32_t baud) { return usart_min_hz(baud); }
    [[gnu::always_inline]] static constexpr uint32_t min_hz_for_over8(uint32_t baud) {
        return usart_min_hz_over8(baud);
    }

    // ---- CR1's own fields, all enable-protected ---------------------------------

    /// CR1.OVER8 (33.5.7): eight samples a bit instead of sixteen, which
    /// halves the minimum divisor - and the tolerance with it (tables
    /// 188/189). BRR must be RE-COMPUTED when this moves: the register's
    /// meaning changes, not just its value.
    static bool oversampling(bool over8) {
        if (enabled()) {
            return false;
        }
        regs().CR1 = over8 ? (regs().CR1 | USART_CR1_OVER8)
                           : (regs().CR1 & ~USART_CR1_OVER8);
        return true;
    }
    static bool oversampling() { return (regs().CR1 & USART_CR1_OVER8) != 0u; }

    /// CR1.FIFOEN (33.5.4). Refused on a BASIC instance, where the bit
    /// is Reserved - and where writing it reads back clear anyway,
    /// which is measured. And turning it ON is refused in LIN or IrDA
    /// mode: 33.8.1's note on the bit, "it must not be enabled in IrDA
    /// and LIN modes" (lin() and irda() refuse the other order).
    static bool fifo(bool on) {
        if (enabled() || !is_full) {
            return false;
        }
        if (on && ((regs().CR2 & USART_CR2_LINEN) != 0u ||
                   (regs().CR3 & USART_CR3_IREN) != 0u)) {
            return false;
        }
        regs().CR1 = on ? (regs().CR1 | USART_CR1_FIFOEN)
                        : (regs().CR1 & ~USART_CR1_FIFOEN);
        return true;
    }
    static bool fifo() { return (regs().CR1 & USART_CR1_FIFOEN) != 0u; }

    /// CR3.RXFTCFG / TXFTCFG. `none` leaves that side's code alone.
    static bool fifo_thresholds(UartFifoThreshold rx, UartFifoThreshold tx) {
        if (enabled() || !is_full ||
            !uart_fifo_threshold_valid(rx) || !uart_fifo_threshold_valid(tx)) {
            return false;
        }
        uint32_t cr3 = regs().CR3;
        if (rx != UartFifoThreshold::none) {
            cr3 = (cr3 & ~USART_CR3_RXFTCFG) |
                  (static_cast<uint32_t>(rx) << USART_CR3_RXFTCFG_Pos);
        }
        if (tx != UartFifoThreshold::none) {
            cr3 = (cr3 & ~USART_CR3_TXFTCFG) |
                  (static_cast<uint32_t>(tx) << USART_CR3_TXFTCFG_Pos);
        }
        regs().CR3 = cr3;
        return true;
    }

    /// CR1.MME/WAKE/ADDM7 + CR2.ADD (33.5.10). ADD is SHARED with the
    /// character match of 33.5.11 - the same eight bits mean the mute
    /// address here and the matched character there - so a program
    /// cannot have both at two different values, and this driver does
    /// not pretend it can.
    static bool mute_mode(const MuteConfig& c) {
        if (enabled()) {
            return false;
        }
        uint32_t cr1 = regs().CR1 | USART_CR1_MME;
        cr1 = c.wake == MuteWake::address_mark ? (cr1 | USART_CR1_WAKE)
                                               : (cr1 & ~USART_CR1_WAKE);
        regs().CR1 = cr1;
        uint32_t cr2 = regs().CR2 & ~(USART_CR2_ADD | USART_CR2_ADDM7);
        if (c.address_7bit) {
            cr2 |= USART_CR2_ADDM7;
        }
        regs().CR2 = cr2 | (static_cast<uint32_t>(c.address) << USART_CR2_ADD_Pos);
        return true;
    }
    static bool mute_mode_off() {
        if (enabled()) {
            return false;
        }
        regs().CR1 = regs().CR1 & ~USART_CR1_MME;
        return true;
    }
    static bool muted() { return (regs().ISR & UsartFlag::rwu) != 0u; }

    /// The character-match half of the SAME CR2.ADD field (33.5.11's
    /// Modbus/ASCII: put LF there and CMF marks the end of a block). The
    /// full eight bits are compared when mute mode is not running.
    static bool character_match(uint8_t ch) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = (regs().CR2 & ~USART_CR2_ADD) |
                     (static_cast<uint32_t>(ch) << USART_CR2_ADD_Pos);
        return true;
    }

    /// CR1.DEAT/DEDT + CR3.DEM/DEP (33.5.20). The DE signal comes out on
    /// the RTS pad, so the caller hands that pad to the peripheral.
    static bool driver_enable(const DriverEnableConfig& c) {
        if (enabled() || !driver_enable_valid(c)) {
            return false;
        }
        regs().CR1 = (regs().CR1 & ~(USART_CR1_DEAT | USART_CR1_DEDT)) |
                     (static_cast<uint32_t>(c.assertion) << USART_CR1_DEAT_Pos) |
                     (static_cast<uint32_t>(c.deassertion) << USART_CR1_DEDT_Pos);
        uint32_t cr3 = regs().CR3 | USART_CR3_DEM;
        cr3 = c.active_low ? (cr3 | USART_CR3_DEP) : (cr3 & ~USART_CR3_DEP);
        regs().CR3 = cr3;
        return true;
    }
    static bool driver_enable_off() {
        if (enabled()) {
            return false;
        }
        regs().CR3 = regs().CR3 & ~USART_CR3_DEM;
        return true;
    }

    // ---- CR2's own fields --------------------------------------------------------

    /// CR2.SWAP: TX and RX exchange PADS (not signals) - a crossed cable
    /// undone in the chip.
    static bool swap(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = on ? (regs().CR2 | USART_CR2_SWAP) : (regs().CR2 & ~USART_CR2_SWAP);
        return true;
    }
    static bool swap() { return (regs().CR2 & USART_CR2_SWAP) != 0u; }

    /// CR2.TXINV/RXINV/DATAINV. The first two invert the LINE's idle and
    /// active levels; the third inverts the DATA (and, 33.5.17 says, the
    /// parity bit with it) - which is the smartcard inverse convention's
    /// other half.
    static bool invert(bool tx, bool rx, bool data) {
        if (enabled()) {
            return false;
        }
        uint32_t cr2 = regs().CR2 &
                       ~(USART_CR2_TXINV | USART_CR2_RXINV | USART_CR2_DATAINV);
        if (tx) { cr2 |= USART_CR2_TXINV; }
        if (rx) { cr2 |= USART_CR2_RXINV; }
        if (data) { cr2 |= USART_CR2_DATAINV; }
        regs().CR2 = cr2;
        return true;
    }

    /// CR2.MSBFIRST: the data bits go out most significant first.
    static bool msb_first(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = on ? (regs().CR2 | USART_CR2_MSBFIRST)
                        : (regs().CR2 & ~USART_CR2_MSBFIRST);
        return true;
    }

    /// LIN mode (33.5.13). 8-bit words only, and the company the chapter
    /// forbids is REFUSED here rather than written: STOP must be 00 and
    /// CLKEN, SCEN, HDSEL and IREN clear. OVER8 too - equation 2 of
    /// 33.5.7 gives LIN, smartcard and IrDA the OVER8 = 0 formula alone.
    /// And FIFOEN, which 33.8.1 forbids in LIN mode - a Uart leaves it
    /// set by default on a FULL instance, so the order matters here.
    static bool lin(const LinConfig& c) {
        if (enabled() || !has_lin_mode) {
            return false;
        }
        const uint32_t cr1 = regs().CR1;
        const uint32_t cr2 = regs().CR2;
        const uint32_t cr3 = regs().CR3;
        if ((cr1 & (USART_CR1_M0 | USART_CR1_M1 | USART_CR1_OVER8 | USART_CR1_FIFOEN)) != 0u ||
            (cr2 & (USART_CR2_STOP | USART_CR2_CLKEN)) != 0u ||
            (cr3 & (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN)) != 0u) {
            return false;
        }
        uint32_t v = cr2 | USART_CR2_LINEN;
        v = c.break_11bit ? (v | USART_CR2_LBDL) : (v & ~USART_CR2_LBDL);
        v = c.break_interrupt ? (v | USART_CR2_LBDIE) : (v & ~USART_CR2_LBDIE);
        regs().CR2 = v;
        return true;
    }
    static bool lin_off() {
        if (enabled()) {
            return false;
        }
        regs().CR2 = regs().CR2 & ~(USART_CR2_LINEN | USART_CR2_LBDIE);
        return true;
    }

    /// Synchronous master (33.5.14): CLKEN + CPOL/CPHA/LBCL. The same
    /// exclusions as LIN's, the other way round.
    static bool synchronous(const SyncConfig& c) {
        if (enabled() || !has_synchronous_mode) {
            return false;
        }
        const uint32_t cr2 = regs().CR2;
        const uint32_t cr3 = regs().CR3;
        if ((cr2 & USART_CR2_LINEN) != 0u ||
            (cr3 & (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN)) != 0u) {
            return false;
        }
        uint32_t v = cr2 | USART_CR2_CLKEN;
        v = c.clock_idle_high ? (v | USART_CR2_CPOL) : (v & ~USART_CR2_CPOL);
        v = c.sample_second_edge ? (v | USART_CR2_CPHA) : (v & ~USART_CR2_CPHA);
        v = c.last_bit_clock ? (v | USART_CR2_LBCL) : (v & ~USART_CR2_LBCL);
        regs().CR2 = v;
        return true;
    }
    static bool synchronous_off() {
        if (enabled()) {
            return false;
        }
        regs().CR2 = regs().CR2 & ~USART_CR2_CLKEN;
        return true;
    }

    /// CR2.RTOEN + RTOR.RTO (33.5.16): RTOF after `bits` bit times of
    /// silence on the receive line. RTOR is 24 bits and, 33.8.7 says,
    /// may be written on the fly - so the ENABLE is enable-protected and
    /// the VALUE is not, and they are two verbs for that reason.
    static bool receiver_timeout_enable(bool on) {
        if (enabled() || !has_receiver_timeout) {
            return false;
        }
        regs().CR2 = on ? (regs().CR2 | USART_CR2_RTOEN)
                        : (regs().CR2 & ~USART_CR2_RTOEN);
        return true;
    }
    static bool receiver_timeout(uint32_t bits) {
        if (!has_receiver_timeout || bits > 0x00FF'FFFFu) {
            return false;
        }
        regs().RTOR = (regs().RTOR & ~USART_RTOR_RTO) | bits;
        return true;
    }
    static uint32_t receiver_timeout() { return regs().RTOR & USART_RTOR_RTO; }

    /// RTOR.BLEN (33.5.17's T = 1 block mode): EOBF after BLEN + 4
    /// characters.
    static bool block_length(uint8_t blen) {
        if (!has_receiver_timeout) {
            return false;
        }
        regs().RTOR = (regs().RTOR & ~USART_RTOR_BLEN) |
                      (static_cast<uint32_t>(blen) << USART_RTOR_BLEN_Pos);
        return true;
    }

    /// CR2.ABREN + ABRMODE (33.5.9). BRR MUST ALREADY HOLD A NON-ZERO
    /// value - the chapter says so and the unit measures against it - so
    /// this refuses on a zero BRR rather than arming a detector that
    /// cannot converge.
    static bool auto_baud(AutoBaudMode m) {
        if (enabled() || !is_full || m == AutoBaudMode::off) {
            return false;
        }
        if ((regs().BRR & 0xFFFFu) == 0u) {
            return false;
        }
        regs().CR2 = (regs().CR2 & ~USART_CR2_ABRMODE) | USART_CR2_ABREN |
                     (static_cast<uint32_t>(m) << USART_CR2_ABRMODE_Pos);
        return true;
    }
    /// REFUSES with UE set - which is CR2's own protection, and ALSO the
    /// structural answer to ES0418 2.12.4 (a G071/G081 item with no twin
    /// in the G0B1's ES0548): clearing ABREN DURING a reception corrupts
    /// the data being received. This driver has no path to that write -
    /// the instance is disabled first, and a disabled receiver is not
    /// receiving - so the erratum has nothing to act on.
    static bool auto_baud_off() {
        if (enabled()) {
            return false;
        }
        regs().CR2 = regs().CR2 & ~(USART_CR2_ABREN | USART_CR2_ABRMODE);
        return true;
    }
    /// 33.5.9: a new detection is launched by RESETTING ABRF - and the
    /// way to reset it is to write a ONE to RQR.ABRRQ, which clears
    /// ABRF and ABRE together and starts a fresh measurement (33.8.8).
    /// RQR is a write-only strobe register and not the ICR, so this is
    /// the one flag of the chapter that is cleared from somewhere else.
    static void auto_baud_restart() { regs().RQR = USART_RQR_ABRRQ; }

    // ---- CR3's own fields ---------------------------------------------------------

    /// CR3.HDSEL (33.5.15): TX and RX internally connected, the RX pin
    /// unused, the TX pin RELEASED whenever nothing is transmitted. The
    /// pad must be alternate-function OPEN DRAIN with a pull-up - the
    /// task does that when the option is set, and a caller driving the
    /// resource by hand owes it.
    static bool half_duplex(bool on) {
        if (enabled()) {
            return false;
        }
        if (on) {
            const uint32_t cr2 = regs().CR2;
            const uint32_t cr3 = regs().CR3;
            if ((cr2 & (USART_CR2_LINEN | USART_CR2_CLKEN)) != 0u ||
                (cr3 & (USART_CR3_SCEN | USART_CR3_IREN)) != 0u) {
                return false;
            }
            regs().CR3 = cr3 | USART_CR3_HDSEL;
        } else {
            regs().CR3 = regs().CR3 & ~USART_CR3_HDSEL;
        }
        return true;
    }
    static bool half_duplex() { return (regs().CR3 & USART_CR3_HDSEL) != 0u; }

    /// CR3.ONEBIT (33.5.8): one sample a bit instead of the majority of
    /// three. It buys tolerance (tables 188/189) and LOSES the noise
    /// flag, which is a majority-vote disagreement and nothing else.
    static bool one_bit_sampling(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR3 = on ? (regs().CR3 | USART_CR3_ONEBIT)
                        : (regs().CR3 & ~USART_CR3_ONEBIT);
        return true;
    }

    /// CR3.OVRDIS: the receiver stops reporting overruns AND STOPS
    /// KEEPING THE OLD BYTE - a new character overwrites RDR instead of
    /// being dropped. It is not "no more errors", it is "lose the old
    /// one silently instead of the new one loudly".
    static bool overrun_disable(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR3 = on ? (regs().CR3 | USART_CR3_OVRDIS)
                        : (regs().CR3 & ~USART_CR3_OVRDIS);
        return true;
    }

    /// CR3.RTSE/CTSE (33.5.20). RTS is an OUTPUT the receiver asserts
    /// when it cannot take more; CTS is an INPUT the transmitter obeys
    /// between frames. Both ride pads the caller hands over.
    static bool flow_control(bool rts, bool cts) {
        if (enabled()) {
            return false;
        }
        uint32_t cr3 = regs().CR3 & ~(USART_CR3_RTSE | USART_CR3_CTSE);
        if (rts) { cr3 |= USART_CR3_RTSE; }
        if (cts) { cr3 |= USART_CR3_CTSE; }
        regs().CR3 = cr3;
        return true;
    }

    /// Smartcard mode (33.5.17): SCEN, NACK, SCARCNT, the guard time and
    /// the CK prescaler. STOP is set to 1.5 (or 0.5 for a receive-only
    /// arrangement) here, because the standard has no other legal value.
    static bool smartcard(const SmartcardConfig& c) {
        if (enabled() || !is_full || !smartcard_valid(c)) {
            return false;
        }
        const uint32_t cr2 = regs().CR2;
        if ((cr2 & USART_CR2_LINEN) != 0u ||
            (regs().CR3 & (USART_CR3_HDSEL | USART_CR3_IREN)) != 0u) {
            return false;
        }
        uint32_t v = (cr2 & ~USART_CR2_STOP) |
                     (static_cast<uint32_t>(c.receive_half_stop ? UartStop::half
                                                                : UartStop::one_and_half)
                      << USART_CR2_STOP_Pos);
        v = c.clock_output ? (v | USART_CR2_CLKEN) : (v & ~USART_CR2_CLKEN);
        regs().CR2 = v;
        regs().GTPR = (static_cast<uint32_t>(c.guard_time) << USART_GTPR_GT_Pos) |
                      static_cast<uint32_t>(c.clock_prescaler);
        uint32_t cr3 = (regs().CR3 & ~USART_CR3_SCARCNT) | USART_CR3_SCEN |
                       (static_cast<uint32_t>(c.retries) << USART_CR3_SCARCNT_Pos);
        cr3 = c.nack ? (cr3 | USART_CR3_NACK) : (cr3 & ~USART_CR3_NACK);
        regs().CR3 = cr3;
        (void)block_length(c.block_length);
        return true;
    }
    static bool smartcard_off() {
        if (enabled()) {
            return false;
        }
        regs().CR3 = regs().CR3 & ~(USART_CR3_SCEN | USART_CR3_NACK | USART_CR3_SCARCNT);
        regs().CR2 = regs().CR2 & ~(USART_CR2_STOP | USART_CR2_CLKEN);
        regs().GTPR = 0;
        return true;
    }
    /// 33.8.4: SCARCNT may be written to ZERO with the USART enabled -
    /// the one documented escape from the enable rule in this register -
    /// and its purpose is to STOP a retransmission in progress.
    static void stop_retries() { regs().CR3 = regs().CR3 & ~USART_CR3_SCARCNT; }

    /// IrDA (33.5.18): IREN + IRLP + the GTPR.PSC divisor, which is both
    /// the low-power baud divisor and the receive glitch filter. Stop
    /// bits must be 1 and LINEN/CLKEN/SCEN/HDSEL clear - refused, not
    /// written over - and FIFOEN clear, 33.8.1's note on that bit.
    static bool irda(const IrdaConfig& c) {
        if (enabled() || !is_full || !irda_valid(c)) {
            return false;
        }
        const uint32_t cr2 = regs().CR2;
        if ((regs().CR1 & USART_CR1_FIFOEN) != 0u ||
            (cr2 & (USART_CR2_LINEN | USART_CR2_STOP | USART_CR2_CLKEN)) != 0u ||
            (regs().CR3 & (USART_CR3_SCEN | USART_CR3_HDSEL)) != 0u) {
            return false;
        }
        regs().GTPR = (regs().GTPR & USART_GTPR_GT) | static_cast<uint32_t>(c.prescaler);
        uint32_t cr3 = regs().CR3 | USART_CR3_IREN;
        cr3 = c.low_power ? (cr3 | USART_CR3_IRLP) : (cr3 & ~USART_CR3_IRLP);
        regs().CR3 = cr3;
        return true;
    }
    static bool irda_off() {
        if (enabled()) {
            return false;
        }
        regs().CR3 = regs().CR3 & ~(USART_CR3_IREN | USART_CR3_IRLP);
        return true;
    }

    /// CR1.UESM + CR3.WUS/WUFIE (33.5.21) and the EXTI line that carries
    /// the wake out of the peripheral. The obligations this verb CANNOT
    /// enforce, and which the caller owes: no transfer ongoing when Stop
    /// is entered, REACK checked after the enable, DMA reception
    /// disabled first, and a kernel clock that survives Stop - HSI16 or
    /// LSE, since PCLK and SYSCLK do not.
    ///
    /// AND ONE MORE THE CALLER OWES BECAUSE WUF IS A LEVEL: with WUFIE
    /// set, a handler that does not clear WUF re-enters for ever - the
    /// ORE storm one flag along. The task's isr() clears it when the
    /// option is named; a program arming the wake through the RESOURCE,
    /// under a task not compiled for it, must clear WUF itself. Measured
    /// the hard way on this stratum's own bench, by halt-and-dump.
    ///
    /// ES0548 2.2.4 applies here and nowhere this driver can help: with
    /// HSIDIV != 0 the wake does not happen at all (measured), and
    /// RCC_CR.HSIKERON does not rescue it.
    static bool wake_from_stop(UsartWakeSource s) {
        if (enabled() || exti_line == 0xFF) {
            return false;
        }
        if (s == UsartWakeSource::none) {
            regs().CR1 = regs().CR1 & ~USART_CR1_UESM;
            regs().CR3 = regs().CR3 & ~(USART_CR3_WUS | USART_CR3_WUFIE);
            return true;
        }
        regs().CR1 = regs().CR1 | USART_CR1_UESM;
        regs().CR3 = (regs().CR3 & ~USART_CR3_WUS) | USART_CR3_WUFIE |
                     (static_cast<uint32_t>(s) << USART_CR3_WUS_Pos);
        return true;
    }
    /// The EXTI mask bit the wake needs to reach the NVIC. The line is
    /// DIRECT (table 65): no sense to choose and no pending bit of the
    /// EXTI's own - WUF is the pending state.
    static bool wake_line(bool on) {
        if (exti_line == 0xFF) {
            return false;
        }
        return Exti::interrupt(exti_line, on);
    }

    /// The PRESC prescaler (33.8.14). A Reserved code is REFUSED, not
    /// written: the chapter's own note says the silicon turns one into
    /// 1011 (divide by 256), which is a rate no caller asked for.
    static bool prescaler(UsartPrescaler p) {
        if (enabled() || !has_prescaler || !usart_prescaler_valid(p)) {
            return false;
        }
        regs().PRESC = static_cast<uint32_t>(p);
        return true;
    }
    static UsartPrescaler prescaler() {
        return static_cast<UsartPrescaler>(regs().PRESC & USART_PRESC_PRESCALER_Msk);
    }

    /// GTPR, raw - the guard time and the prescaler share one register
    /// and mean different things per mode (smartcard vs IrDA).
    static bool guard_time(uint8_t gt) {
        if (enabled()) {
            return false;
        }
        regs().GTPR = (regs().GTPR & ~USART_GTPR_GT) |
                      (static_cast<uint32_t>(gt) << USART_GTPR_GT_Pos);
        return true;
    }

    // ---- requests, data, flags ----------------------------------------------------

    /// USART_RQR (33.8.8): a write-only strobe, legal while enabled -
    /// which is the whole point of the register.
    static void request(UsartRequest r) { regs().RQR = static_cast<uint32_t>(r); }
    /// 33.5.13: with LINEN set, SBKRQ sends 13 zero bits and then two
    /// ones; without it, one break character of the frame's own length.
    static void send_break() { request(UsartRequest::send_break); }

    static uint32_t status() { return regs().ISR; }
    static bool flag(uint32_t mask) { return (regs().ISR & mask) != 0u; }
    static void clear_flags(uint32_t icr_mask) { regs().ICR = icr_mask; }
    static uint8_t read_data() { return static_cast<uint8_t>(regs().RDR); }
    /// The full 9-bit datum, for a frame that carries one.
    static uint16_t read_word() { return static_cast<uint16_t>(regs().RDR & 0x1FFu); }
    static void write_data(uint8_t b) { regs().TDR = b; }
    static void write_word(uint16_t v) { regs().TDR = v & 0x1FFu; }

    /// Where a DMA channel reads from and writes to. TWO REGISTERS, not
    /// one: a receive engine points at RDR and a transmit engine at TDR,
    /// and neither can be mistaken for the other.
    static volatile void* tx_data_address() { return &regs().TDR; }
    static volatile void* rx_data_address() { return &regs().RDR; }

    /**
     * CR3.DMAT / CR3.DMAR (33.8.4): whether the peripheral raises a DMA
     * REQUEST for the condition, instead of - or as well as - the
     * interrupt. With the bit set, TXE (or RXNE) drives the request line
     * and the channel clears the condition by writing (or reading) the
     * data register, so the matching interrupt must NOT also be armed:
     * both would serve the same byte.
     *
     * NOT UE-PROTECTED: 33.8.4 gives the "only when UE = 0" clause bit by
     * bit, and DMAT and DMAR carry none, so either is written with the
     * instance running. Not part of configure() all the same - an engine
     * turns its bit on once the frame is settled, and an application
     * without one never touches them. CR3 is read-modify-written here
     * and by the receive engine's vector (rx_request_restart()), hence the
     * guard.
     */
    static void dma_transmit(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_DMAT) : (regs().CR3 & ~USART_CR3_DMAT);
    }
    static void dma_receive(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_DMAR) : (regs().CR3 & ~USART_CR3_DMAR);
    }

    /**
     * DMAR dropped and raised again: THE RECEIVE REQUEST PRESENTED ANEW.
     * The cure for this silicon's FIFO wedge (docs/stm32g0/usart.md, "The
     * receive FIFO under a starved channel"): after a starvation of a
     * little over a character time, a receive FIFO holding entries can
     * keep its request standing while the channel takes no further one,
     * and nothing but the request seen to fall restarts it - the DMAMUX
     * routed away and back does the same, a cleared flag or a new
     * character does not. Legal with UE set (33.8.4, above); the
     * bracketed read and two stores, under a guard because CR3 is shared
     * with the main context's verbs.
     */
    static void rx_request_restart() {
        InterruptGuard guard;
        const uint32_t cr3 = regs().CR3;
        regs().CR3 = cr3 & ~USART_CR3_DMAR;
        regs().CR3 = cr3;
    }

    /**
     * THE DMAMUX REQUEST LINES THIS INSTANCE PUBLISHES (RM0444 table 55).
     *
     * They live here and not in stm32g0/device_tables.hpp because NO
     * DEVICE HEADER OF THIS PACK DECLARES THEM - the DMAMUX_REQ_*
     * spellings are ST's HAL/LL, which this project does not vendor - and
     * because a fabric driver owns the fabric while a peripheral owns
     * its own vocabulary. stm32g0/dma.hpp therefore takes a plain
     * request id and knows nothing about USARTs.
     *
     * The numbers are the same on every part of the family; an instance
     * a part does not bond simply has no user for its row.
     */
    static constexpr uint8_t dma_rx_request() {
        switch (n) {
            case 1: return 50;
            case 2: return 52;
            case 3: return 54;
            case 4: return 56;
            case 5: return 74;
            default: return 76;
        }
    }
    static constexpr uint8_t dma_tx_request() {
        switch (n) {
            case 1: return 51;
            case 2: return 53;
            case 3: return 55;
            case 4: return 57;
            case 5: return 75;
            default: return 77;
        }
    }

    // ---- interrupt enables ---------------------------------------------------------

    static void rxne_interrupt(bool on) {
        InterruptGuard guard;   // CR1 is read-modify-write and shared with the handler
        regs().CR1 = on ? (regs().CR1 | USART_CR1_RXNEIE_RXFNEIE)
                        : (regs().CR1 & ~USART_CR1_RXNEIE_RXFNEIE);
    }
    static void txe_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR1 = on ? (regs().CR1 | USART_CR1_TXEIE_TXFNFIE)
                        : (regs().CR1 & ~USART_CR1_TXEIE_TXFNFIE);
    }
    static bool txe_interrupt() { return (regs().CR1 & USART_CR1_TXEIE_TXFNFIE) != 0u; }

    /// Any set of the CR1 interrupt enables at once (UsartInterrupt).
    static void interrupts(uint32_t cr1_mask, bool on) {
        InterruptGuard guard;
        regs().CR1 = on ? (regs().CR1 | cr1_mask) : (regs().CR1 & ~cr1_mask);
    }
    static uint32_t interrupts() { return regs().CR1; }

    /// CR3's four: the error interrupt (EIE - FE/ORE/NE while DMA
    /// reception is on), CTS, and the two FIFO thresholds.
    static void error_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_EIE) : (regs().CR3 & ~USART_CR3_EIE);
    }
    static void cts_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_CTSIE) : (regs().CR3 & ~USART_CR3_CTSIE);
    }
    static void rx_threshold_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_RXFTIE) : (regs().CR3 & ~USART_CR3_RXFTIE);
    }
    static void tx_threshold_interrupt(bool on) {
        InterruptGuard guard;
        regs().CR3 = on ? (regs().CR3 | USART_CR3_TXFTIE) : (regs().CR3 & ~USART_CR3_TXFTIE);
    }

    /// Software reset of the block through RCC (APBRSTR1/2): every
    /// register back to its reset value.
    static void reset() {
        constexpr UsartBusClock bc = usart_bus_clock(n);
        if constexpr (bc.apb2) {
            RCC->APBRSTR2 = RCC->APBRSTR2 | bc.mask;
            RCC->APBRSTR2 = RCC->APBRSTR2 & ~bc.mask;
        } else {
            RCC->APBRSTR1 = RCC->APBRSTR1 | bc.mask;
            RCC->APBRSTR1 = RCC->APBRSTR1 & ~bc.mask;
        }
    }
};

// ---- the task -----------------------------------------------------------------

/*
 * The "no DMA engine" default of the two optional Uart engine slots is
 * `NoDmaEngine`, and it lives in stm32g0/dma_engine.hpp (included
 * above): spi.hpp's own two slots take the same tag, and two headers
 * cannot each define it. That file carries the reasoning - a tag, not a
 * base class; a slot that folds to nothing; and why a driver with an
 * optional engine must never include stm32g0/dma.hpp.
 */

/// Two engines on one transport must not name the same DMA channel: a
/// channel moves data ONE way, and pointing both directions at it would
/// have each re-programming the other's block. Generic over any engine
/// that says `present`, `controller` and `channel` - `if constexpr` keeps
/// those from being looked up on an absent engine, which is what lets
/// NoDmaEngine stay a two-line tag.
template <typename Tx, typename Rx>
constexpr bool uart_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::controller != Rx::controller || Tx::channel != Rx::channel;
    } else {
        return true;
    }
}

/**
 * UartOptions::fifo - CR1.FIFOEN as the task's option, in THREE states:
 * stated on (`.fifo = true`, refused at compile time on an instance with
 * no FIFO), stated off (`.fifo = false`), and UNSTATED, the default, in
 * which the task decides from the instance (UartTask::fifo_mode says
 * how). A bool cannot tell "off" from "not said", and std::optional<bool>
 * is not a structural type, which a template argument must be - so this
 * is a struct a bool converts to IMPLICITLY, on purpose: every program
 * that wrote `.fifo = true` or `.fifo = false` still means what it meant.
 */
struct UartFifo {
    bool stated = false;
    bool on = false;
    constexpr UartFifo() = default;
    constexpr UartFifo(bool v) : stated(true), on(v) {}
};

/**
 * EVERYTHING THE CHAPTER OFFERS A BYTE TRANSPORT, as one constexpr
 * struct the task takes as its LAST template argument - and every member
 * of it is `if constexpr`-ed below, so a feature nobody names costs
 * nothing.
 *
 * THE DEFAULT IS NOT THE ONE-REGISTER SHAPE: with `fifo` unstated the
 * task turns the FIFO on wherever the instance has one and paces the
 * transmitter on the FIFO running EMPTY, one interrupt refilling eight
 * characters, because that is the better transport on this silicon
 * (docs/stm32g0/usart.md has the numbers). A program that wants one
 * character per interrupt says `.fifo = false`.
 *
 * WHY AN OPTIONS STRUCT AND NOT TEN TEMPLATE PARAMETERS: the surface of
 * Uart is a SHARED one (util/serial_port.hpp and print() compile
 * against every target's), so its parameter list cannot grow a
 * target-specific tail. One trailing NTTP with a default is the whole
 * change, and a caller who names nothing sees nothing.
 */
struct UartOptions {
    /// CCIPR: which clock feeds the baud generator. A kernel clock of
    /// hsi16 or lse DOES NOT MOVE WITH SYSCLK, so rebase() rewrites
    /// nothing for one - which is the point of having one.
    UsartClock kernel_clock = UsartClock::pclk;
    UsartPrescaler prescaler = UsartPrescaler::div1;
    bool over8 = false;

    /// FIFO mode (33.5.4): unstated, the task decides (UartFifo above,
    /// UartTask::fifo_mode). The thresholds are written when the FIFO is
    /// on and they are not `none`.
    ///
    /// THE TRANSMITTER RIDES TXFT, by default at code 101, "TXFIFO
    /// becomes empty" (33.8.4): the flag rises as the shift register
    /// takes the last entry, so the handler refills all eight places
    /// while that character still has a whole frame to go - one
    /// interrupt per eight characters, and no gap on the wire unless the
    /// handler is a frame late. A code below 101 trades interrupts for
    /// slack against a late handler; `none` rides TXFNF, which under a
    /// long print tops the FIFO up one character an entry - measured
    /// dearer per character than the FIFO left off. A transmit threshold
    /// never leaves a tail behind: the FIFO drains on its own, so the
    /// condition always comes.
    ///
    /// THE RECEIVER RIDES RXFT, by default at code 010, half the FIFO
    /// (33.8.4): one entry drains four characters or more, and a handler
    /// late by five frames still loses nothing (four places and the shift
    /// register). A threshold alone would strand the tail below it, so
    /// the line's own end-of-burst condition serves the tail - the
    /// RECEIVER TIME-OUT where the instance has one (33.5.16: RTOF after
    /// `rx_timeout` bit times of silence past the last stop bit, a FULL
    /// USART's), the IDLE flag where it has not (an LPUART: one idle
    /// frame, set once a burst - 33.8.10's note keeps it from rising
    /// again before a character has). One entry per level plus one a
    /// burst, where RXFNE costs one per character. `none` keeps RXFNE -
    /// every character its own entry, the shape for a receiver that
    /// must answer each byte at once.
    ///
    /// UNDER A RECEIVE ENGINE THE RECEIVE THRESHOLD IS THE TASK'S: seven
    /// eighths, the FIFO wedge's watch (UartTask, "THE FIFO STAYS ON UNDER
    /// A RECEIVE ENGINE"), and `rx_threshold` left at its default - a
    /// value named there is refused, since nothing would read it.
    UartFifo fifo{};
    UartFifoThreshold rx_threshold = UartFifoThreshold::half;
    UartFifoThreshold tx_threshold = UartFifoThreshold::full_or_empty;
    /// RTOR.RTO in bit times (33.8.7, 24 bits): how much silence ends a
    /// burst for the paced receiver. Ten is an 8N1 frame's length - the
    /// idle frame an LPUART waits for -, so the tail arrives within two
    /// frames of the last stop bit; RTOF rises within two samples of it
    /// (33.8.10's note). A pause that long inside a stream costs one entry
    /// that finds what the FIFO holds, nothing more.
    uint32_t rx_timeout = 10;

    /// The pad options of 33.8.3.
    bool swap = false;
    bool invert_tx = false;
    bool invert_rx = false;
    bool invert_data = false;
    bool msb_first = false;

    /// 33.5.15: one wire on the TX pad, open drain with a pull-up. The
    /// RX pad of `pins` is then NOT claimed, and every byte transmitted
    /// comes back through the receiver.
    bool half_duplex = false;

    /// 33.5.8 / 33.8.4: one sample a bit (more tolerance, no noise flag)
    /// and the overrun that overwrites instead of dropping.
    bool one_bit = false;
    bool overrun_disable = false;

    /// RS-485 driver enable (33.5.20). The DE signal comes out on the
    /// RTS pad, which is why the pad travels in the options.
    bool driver_enable = false;
    PinSel de_pin{};
    uint8_t de_assertion = 0;
    uint8_t de_deassertion = 0;
    bool de_active_low = false;

    /// RS-232 flow control (33.5.20). RTS and CTS are separate pads and
    /// separate enables; naming a pad without its enable claims nothing.
    bool rts = false;
    bool cts = false;
    PinSel rts_pin{};
    PinSel cts_pin{};

    /// 33.5.21: what wakes the core out of Stop through this instance.
    UsartWakeSource wake_from_stop = UsartWakeSource::none;

    /// The receive engine's level in the DMA arbitration (RM0444 10.4.4's
    /// CCR.PL), `very_high` by default: RDR and the FIFO behind it hold
    /// nine characters at most, and a channel starved past them loses the
    /// next one to an overrun - which two memory-to-memory copies, each
    /// alternating with the other, do to a ring ranked below them
    /// (docs/design/dma.md). A program that ranks another channel above
    /// its ring says so here; refused on a port with no receive engine,
    /// which has no channel to rank.
    DmaPriority rx_priority = DmaPriority::very_high;
};

/// A copy of `base` with the RS-485 driver enable filled in - the maker
/// the Rs485 alias below uses, so an application states the pad and the
/// timings once.
constexpr UartOptions uart_with_driver_enable(UartOptions base, PinSel de,
                                              uint8_t assertion = 0,
                                              uint8_t deassertion = 0,
                                              bool active_low = false) {
    base.driver_enable = true;
    base.de_pin = de;
    base.de_assertion = assertion;
    base.de_deassertion = deassertion;
    base.de_active_low = active_low;
    return base;
}

/// A copy of `base` in single-wire half-duplex. It is an OPTION and not
/// a task of its own, because 33.5.15 is a bit and not a mode.
constexpr UartOptions uart_half_duplex(UartOptions base = {}) {
    base.half_duplex = true;
    return base;
}

/**
 * The receive ring a transport keeps, chosen by whether its receive has
 * an engine: Ring, which the handler pushes into, where it has none, and
 * where it has one the CONSUMER HALF of a ring the channel itself writes
 * - util/ring.hpp's HardwareRing over a storage array the transport owns
 * (keyed by the transport's type), the receive engine its RingCounter.
 * A partial specialization and not std::conditional_t, because naming
 * HardwareRing over NoDmaEngine would be checked even in the arm not
 * taken, and NoDmaEngine counts nothing.
 */
template <bool engine, typename Owner, uint32_t size, typename Engine>
struct UartRxRing {
    using type = SkipRing<uint8_t, size, Stm32g0Platform<>>;
};
template <typename Owner, uint32_t size, typename Engine>
struct UartRxRing<true, Owner, size, Engine> {
    static inline uint8_t storage[size]{};
    using type = HardwareRing<storage, Engine>;
};

/**
 * UartTask<Resource, pins, rx_size, tx_size, TxEngine, RxEngine, opts>:
 * the interrupt-driven byte transport, over a USART or an LPUART.
 *
 *   constexpr brio::UartPins console_pins{
 *       .tx = {'A', 2, brio::PinFunction::af1},   // USART2_TX (DS13560 table 13)
 *       .rx = {'A', 3, brio::PinFunction::af1},   // USART2_RX
 *   };
 *   using Serial = brio::Uart<2, console_pins>;
 *   extern "C" void USART2_LPUART2_IRQHandler() {
 *       if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
 *   }
 *   Serial::init(clock, 115200);
 *
 * The shared Uart surface - same verbs, same return contracts:
 * init/rebase/isr/write_byte/write_bulk/read_byte/read_bulk/read_span/
 * consume, the error counters, release().
 *
 * ONE IMPLEMENTATION, TWO PERIPHERALS. `Uart<n, ...>` names it over
 * `Usart<n>` and `LpUart<n, ...>` (stm32g0/lpuart.hpp) over `Lpuart<n>`;
 * everything that differs - the baud arithmetic, which features exist,
 * where the kernel-clock field sits, which vector and which EXTI line -
 * the RESOURCE answers. Chapter 34 is chapter 33 with a different
 * divisor, and a second copy of this file would have been a second place
 * for the ORE storm to be got wrong.
 *
 * THE TWO OPTIONAL ENGINE SLOTS: name a stm32g0/dma.hpp DmaTxEngine
 * and/or DmaRxEngine and the bytes move without the CPU; name neither
 * (the default) and every engine branch below disappears - `if
 * constexpr` throughout, so an engineless image carries no DMA code at
 * all (docs/stm32g0/dma.md records the md5 gate).
 *
 * WHICHEVER DIRECTION HAS AN ENGINE DOES NOT ARM ITS INTERRUPT: the DMA
 * request and the interrupt are the SAME condition (TXE, RXNE), so arming
 * both would have the channel and the handler each serve one byte.
 *
 * THE RX ENGINE RUNS A RING, NOT RUNS. The receive engine is armed in
 * its CIRCULAR shape over the whole receive ring's storage
 * (stm32g0/dma.hpp's DmaRxEngine): the channel writes the storage lap
 * after lap and is never re-armed, the ring's producer index is the
 * channel's own count, and the receive ring is the consumer half of that
 * (util/ring.hpp's HardwareRing) - so a byte that lands is readable at
 * once, a burst longer than the run to the end of the storage wraps with
 * no CPU, and nothing is lost between runs because there are none. What
 * the consumer did not keep up with - a lap written over its unread bytes
 * - is counted and skipped by the ring, and rx_overruns() reports it.
 *
 * THE RECEIVE EDGE COMES FROM A VECTOR, never from a poll. Without an
 * engine the receiver rides RXFT with the line's end-of-burst flag for
 * the tail (UartOptions), and isr() reports the ring's empty -> non-empty
 * transition. With one, the line's IDLE enters isr() and the ring's half
 * and full marks enter dma_isr(); both run harvest(), whose answer is the
 * edge - so a burst is told within an idle frame of its last stop bit,
 * and a stream that never pauses twice a lap.
 *
 * WHAT AN RX ENGINE TRADES AWAY, and it cannot be given back: per-byte
 * error attribution. With the interrupt receiver ISR is read for EACH
 * character before its RDR and a framed or parity-failed byte is dropped
 * precisely. With a channel consuming RXNE instead, each error enters the
 * vector on its own (EIE, PEIE) and is counted there; the character it
 * belongs to is not transferred (DDRE clear, 33.8.4), and no flag is ever
 * cleared by a read of RDR - every one has its ICR bit - so the channel
 * loses nothing to a clear.
 *
 * THE RECEIVE ENGINE ARMS AT `rx_priority`, very_high by default, and the
 * transmit engine at high (docs/design/dma.md): a starved receive
 * overruns, a starved transmit leaves the line idle a while.
 *
 * THE FIFO STAYS ON UNDER A RECEIVE ENGINE, WITH A KICK. On this silicon a
 * channel starved for a little over a character time - two entries queued
 * - can leave the receive FIFO WEDGED: entries standing, DMAR set, the
 * channel enabled with its count frozen, and nothing moving until the
 * request is seen to fall; the FIFO then fills and overruns, and the
 * receiver is dead. ST's own HAL wedges the same way on the same board
 * (docs/stm32g0/usart.md has the measurement and what does and does not
 * release it). So isr() watches for it: an entry for the line's IDLE, a
 * receive error, or RXFT - armed at seven eighths, the FIFO one entry and
 * the shift register short of an overrun - that finds RXFNE still set
 * drops DMAR and raises it again (Usart::rx_request_restart()), and the
 * channel takes the standing entries at once with no byte lost. With it
 * the FIFO absorbs any starvation up to the nine characters it and the
 * shift register hold; without the FIFO (`.fifo = false`) the same
 * starvation above one character loses frames. A healthy stream never
 * kicks; rx_kicks() counts the kicks.
 */
template <typename Res, UartPins pins, uint32_t rx_size = 64,
          uint32_t tx_size = 256, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine, UartOptions opts = UartOptions{}>
class UartTask {
    using S = Res;

    /// The divisor type this peripheral's baud generator speaks: sixteen
    /// bits for a USART (33.5.7), twenty for an LPUART (34.4.7). Named
    /// once, so every arithmetic verb below is the same text for both.
    using Divisor = decltype(S::brr_for(uint32_t{}, uint32_t{}));

    static_assert(opts.half_duplex ? uart_pins_valid_single(pins)
                                   : uart_pins_valid(pins),
                  "brio Uart: the two pads must be real pins of present ports, and TX "
                  "and RX cannot be the same pad (in half duplex only TX is claimed)");

    // AN ENGINE IS CHECKED WHERE IT IS NAMED. `sizeof` demands a COMPLETE
    // type, which instantiates the engine right here, at the template
    // argument the application typed - so an engine's own static_asserts
    // (its channel number above all) fire on that line. Without this the
    // engine stays uninstantiated until something first touches it, and a
    // Uart carrying an impossible channel compiles perfectly happily.
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from stm32g0/dma.hpp, or NoDmaEngine (the default)");
    static_assert(uart_engines_distinct<TxEngine, RxEngine>(),
                  "the transmit and receive engines must use DIFFERENT DMA "
                  "channels: a channel moves data in one direction only");
    static_assert(RxEngine::present || opts.rx_priority == DmaPriority::very_high,
                  "brio Uart: rx_priority ranks the receive engine's channel - this port "
                  "has no receive engine");
    static_assert(!RxEngine::present || opts.rx_threshold == UartFifoThreshold::half,
                  "brio Uart: under a receive engine the receive FIFO threshold is the "
                  "task's (seven eighths, the wedge's watch - docs/stm32g0/usart.md); "
                  "rx_threshold is the interrupt receiver's pace");

    // The instance-capability refusals of table 183/184, at the line the
    // application typed them on.
    static_assert(!(opts.fifo.stated && opts.fifo.on) || S::has_fifo_mode,
                  "brio Uart: FIFO mode is a FULL instance's (RM0444 table 184); this "
                  "instance has none, and FIFOEN would not even stick");
    static_assert(opts.prescaler == UsartPrescaler::div1 || S::has_prescaler,
                  "brio Uart: the PRESC prescaler is a FULL instance's (table 184, and "
                  "ES0548 2.11.2 on the manual revisions that omit the split)");
    static_assert(!opts.over8 || S::has_oversampling8,
                  "brio Uart: the LPUART has no OVER8 - its baud generator is "
                  "256 x fck / LPUARTDIV and nothing else (RM0444 34.4.7)");
    static_assert(opts.kernel_clock == UsartClock::pclk || S::has_clock_select,
                  "brio Uart: this instance has no kernel-clock multiplexer (RCC_CCIPR "
                  "carries one for USART1..3 and the LPUARTs only); it runs on PCLK");
    static_assert(opts.wake_from_stop == UsartWakeSource::none || S::exti_line != 0xFF,
                  "brio Uart: this instance cannot wake the core from Stop (table 184's "
                  "last row); there is no EXTI line for it");
    static_assert(opts.wake_from_stop == UsartWakeSource::none ||
                      opts.kernel_clock == UsartClock::hsi16 ||
                      opts.kernel_clock == UsartClock::lse,
                  "brio Uart: a wake from Stop needs a kernel clock that survives Stop - "
                  "HSI16 or LSE (RM0444 33.5.21). PCLK and SYSCLK are both stopped, so "
                  "the receiver would never see the start bit");
    static_assert(uart_fifo_threshold_valid(opts.rx_threshold) &&
                      uart_fifo_threshold_valid(opts.tx_threshold),
                  "brio Uart: a FIFO threshold code is 000..101 (33.8.4); the remaining "
                  "combinations are Reserved");
    static_assert(opts.rx_timeout >= 1u && opts.rx_timeout <= 0x00FF'FFFFu,
                  "brio Uart: RTOR.RTO is 24 bits of bit times, and a time-out of none "
                  "would raise RTOF at every character (33.8.7)");
    static_assert(usart_prescaler_valid(opts.prescaler),
                  "brio Uart: a Reserved PRESC code (33.8.14) - and the silicon turns "
                  "one into 1011, divide by 256, rather than ignoring it");
    static_assert(driver_enable_valid(DriverEnableConfig{opts.de_assertion,
                                                         opts.de_deassertion,
                                                         opts.de_active_low}),
                  "brio Uart: DEAT and DEDT are five bits each - 0..31 SAMPLE times "
                  "(1/16 of a bit at OVER8 = 0, 1/8 at OVER8 = 1)");
    static_assert(!opts.driver_enable || opts.de_pin.valid(),
                  "brio Uart: the driver-enable signal comes out on the RTS pad, so it "
                  "needs one (RM0444 33.5.20)");
    static_assert(!opts.rts || opts.rts_pin.valid(),
                  "brio Uart: RTS flow control needs the RTS pad");
    static_assert(!opts.cts || opts.cts_pin.valid(),
                  "brio Uart: CTS flow control needs the CTS pad");
    static_assert(!(opts.driver_enable && opts.rts),
                  "brio Uart: DE and RTS are the SAME pad and the same driver (33.5.20) "
                  "- an instance does one or the other");

    using TxPin = Pin<pins.tx.port, pins.tx.pin>;
    using RxPin = Pin<pins.rx.port, pins.rx.pin>;

    // The receive ring: without an engine a SkipRing, told by the handler
    // that it lost a character; the view over the channel's ring with one
    // (UartRxRing above).
    using RxRing = UartRxRing<RxEngine::present, UartTask, rx_size, RxEngine>;
    static_assert(!RxEngine::present || rx_size <= 0x8000u,
                  "brio Uart: the receive engine's ring is one circular block, and "
                  "CNDTR counts 65535 elements at most (RM0444 10.6.4)");
    static inline typename RxRing::type m_rx{};
    static inline Ring<uint8_t, tx_size, Stm32g0Platform<>> m_tx{};

    static inline volatile uint8_t m_rx_overruns = 0;   // RX ring full, byte lost
    static inline volatile uint8_t m_frame_errors = 0;  // FE: byte dropped
    static inline volatile uint8_t m_parity_errors = 0; // PE: byte dropped
    static inline volatile uint8_t m_noise_errors = 0;  // NE: byte kept, line suspect
    static inline volatile uint8_t m_hw_overruns = 0;   // ORE: a byte lost in silicon
    // Under a receive engine, the characters the stream lost beside the
    // ring's own skips - one an ORE, an FE or a PE (DDRE clear: the
    // channel never takes them), one a restart after a transfer error -
    // never cleared: rx_skips() is the ring's skips plus this.
    static inline volatile uint32_t m_rx_lost = 0;
    static inline uint32_t m_baud = 0;                  // for rebase()

    /// DMA blocks this transport had to throw away. Touched only from
    /// inside `if constexpr (has_*_engine)` branches, so an engineless
    /// build never reads or writes it.
    static inline volatile uint8_t m_dma_faults = 0;

    /// The receive request presented anew (isr()'s kick, rx_kicks()).
    /// Touched only under a receive engine with the FIFO on.
    static inline volatile uint8_t m_rx_kicks = 0;

    /// EVERY COUNTER SATURATES at 255: a count that wraps reads as a few
    /// errors after a storm of them. One compare on a rare path.
    [[gnu::always_inline]] static void bump(volatile uint8_t& counter) {
        const uint8_t v = counter;
        if (v != 0xFFu) {
            counter = static_cast<uint8_t>(v + 1u);
        }
    }

    /// Whether the consumer has looked at the receive ring and found it
    /// empty since the last harvest() that reported an edge - what makes
    /// harvest()'s answer an EDGE over a ring the channel fills with no
    /// step of ours in between. TWO CONTEXTS: the consumer writes it, the
    /// vectors that call harvest() test it and clear it. The consumer
    /// sets it BEFORE it looks and clears it again when the look found
    /// bytes, so a byte that lands between the look and the store can
    /// only be reported twice - an RxActivity the drain finds nothing
    /// for - and never not at all. Engined only.
    static inline volatile bool m_rx_drained = true;

public:
    constexpr UartTask() = default;

    using Resource = S;

    /// Whether this instantiation carries an engine at all. Every engine
    /// branch below is `if constexpr` on these, so a false one costs
    /// nothing - not a byte, not a branch.
    static constexpr bool has_tx_engine = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;
    static constexpr UartOptions options = opts;

    /// FIFO MODE AS THIS INSTANTIATION RESOLVED IT - what `options.fifo`
    /// alone no longer says. Stated, the option decides (and "on" is
    /// refused above where the instance has no FIFO). Unstated, the task
    /// decides: ON where the instance has the FIFO - table 184 gives it
    /// to the FULL USARTs and to every LPUART, `has_fifo_mode` - EXCEPT
    /// under a wake on an address match, because 33.5.21 says "when the
    /// FIFO is enabled, waking up from low-power mode on address match
    /// is only possible when mute mode is enabled", mute mode is the
    /// resource's and not this task's, and without the FIFO that wake
    /// needs nothing else.
    static constexpr bool fifo_mode =
        opts.fifo.stated ? opts.fifo.on
                         : (S::has_fifo_mode &&
                            opts.wake_from_stop != UsartWakeSource::address_match);
    static_assert(opts.fifo.stated || !fifo_mode || S::has_fifo_mode,
                  "brio Uart: the default FIFO decision must follow the instance's "
                  "presence (RM0444 table 184): FIFOEN does not stick on a BASIC USART");

    /// THE RECEIVER'S PACE as this instantiation resolved it: on RXFT at
    /// `rx_threshold` with the line's end-of-burst flag for the tail
    /// (UartOptions says why), where the FIFO is on, no engine owns the
    /// receive and a threshold is named; otherwise RXFNE, a character an
    /// entry - the one-register instance, and `rx_threshold = none`.
    static constexpr bool rx_paced =
        !RxEngine::present && fifo_mode && opts.rx_threshold != UartFifoThreshold::none;

    /// THE RECEIVE ENGINE'S KICK, where it applies: an engine over the
    /// FIFO. Its watch is RXFT at seven eighths beside IDLE and the
    /// receive errors (the class comment's "THE FIFO STAYS ON UNDER A
    /// RECEIVE ENGINE"); without the FIFO there is no second entry to
    /// wedge behind, and nothing to watch.
    static constexpr bool rx_kick = RxEngine::present && fifo_mode;

    /// The receive threshold CR3.RXFTCFG carries: the interrupt receiver's
    /// pace, or under an engine the wedge's watch.
    static constexpr UartFifoThreshold rx_threshold_in_force =
        RxEngine::present ? UartFifoThreshold::seven_eighths : opts.rx_threshold;

    /// THE END OF A BURST, the flag that delivers the tail and the
    /// engine's edge: the receiver time-out (33.5.16) for the paced
    /// receiver of an instance that has one, IDLE otherwise - an LPUART,
    /// and every receive engine, because a channel can sit on a BASIC
    /// USART or an LPUART and IDLE is the flag table 184 gives every
    /// instance. ISR and ICR carry each at the same position.
    static constexpr uint32_t rx_tail =
        (RxEngine::present || !S::has_receiver_timeout) ? UsartFlag::idle : UsartFlag::rtof;

    /// The rate the baud divisor really divides, for the app's clock -
    /// the kernel clock the options name, through the prescaler they
    /// name. A compile-time constant with a static clock.
    template <typename Clock>
    static constexpr uint32_t kernel_hz() {
        return usart_kernel_hz(usart_kernel_clock_hz<Clock>(opts.kernel_clock),
                               opts.prescaler);
    }

    /// Bring the instance up: bus clock, kernel clock, frame, baud,
    /// the chapter options, pads, the receive interrupt and its NVIC line.
    ///
    /// Call AFTER the main clock is set up and before interrupts are
    /// enabled globally; `clock` is the app's brio::Clock tag, and the
    /// divisor comes from THE KERNEL CLOCK THE OPTIONS NAME - never from
    /// a second statement of the rate, and never from PCLK by assumption.
    /// False when the rate cannot be produced at that clock: the caller
    /// then knows the transport is NOT up, rather than printing into a
    /// ring nothing will drain.
    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, const UartFormat& format = {}) {
        static_assert(clock_follows<Clock, UartTask>(),
                      "this Uart is initialized with a DynamicClock that does not "
                      "list it among its Users: it would keep the old baud after "
                      "a clock change");

        // clock_hz(clock) is SYSCLK = PCLK (the clock task pins the
        // prescalers): a constant for a static clock, the rate in force
        // for a dynamic one; baud_clock() turns it into what the divisor
        // divides through the options' kernel clock and prescaler.
        const uint32_t sys_hz = clock_hz(clock);
        const Divisor reg = plain ? Divisor{usart_brr(sys_hz, baud)}
                                  : divisor_for(baud_clock(sys_hz), baud);
        if (!reg) {
            return false;
        }

        Nvic::disable(S::irq());
        m_rx.clear();
        m_tx.clear();
        clear_errors();
        m_baud = baud;

        S::bus_clock(true);
        S::reset();
        if (!S::kernel_clock(opts.kernel_clock)) {
            return false;
        }
        if (!S::configure(format, *reg)) {
            return false;
        }

        // The chapter options, all of them written with UE still clear -
        // and every one of them elided when it is not asked for, which is
        // what keeps a default Uart byte-identical.
        if constexpr (opts.over8) {
            (void)S::oversampling(true);
        }
        if constexpr (opts.prescaler != UsartPrescaler::div1) {
            (void)S::prescaler(opts.prescaler);
        }
        if constexpr (fifo_mode) {
            (void)S::fifo(true);
            (void)S::fifo_thresholds(rx_threshold_in_force, opts.tx_threshold);
        }
        if constexpr (opts.swap) {
            (void)S::swap(true);
        }
        if constexpr (opts.invert_tx || opts.invert_rx || opts.invert_data) {
            (void)S::invert(opts.invert_tx, opts.invert_rx, opts.invert_data);
        }
        if constexpr (opts.msb_first) {
            (void)S::msb_first(true);
        }
        if constexpr (opts.one_bit) {
            (void)S::one_bit_sampling(true);
        }
        if constexpr (opts.overrun_disable) {
            (void)S::overrun_disable(true);
        }
        if constexpr (opts.half_duplex) {
            (void)S::half_duplex(true);
        }
        if constexpr (opts.driver_enable) {
            (void)S::driver_enable(DriverEnableConfig{opts.de_assertion,
                                                      opts.de_deassertion,
                                                      opts.de_active_low});
        }
        if constexpr (opts.rts || opts.cts) {
            (void)S::flow_control(opts.rts, opts.cts);
        }
        if constexpr (opts.wake_from_stop != UsartWakeSource::none) {
            (void)S::wake_from_stop(opts.wake_from_stop);
            (void)S::wake_line(true);
        }

        // Pads BEFORE the enable: TE's idle frame must land on a pad the
        // peripheral already owns. The RX pad gets a pull-up so an
        // unconnected line reads idle rather than noise.
        if constexpr (opts.half_duplex) {
            // 33.5.15: ONE pad, alternate function OPEN DRAIN with a
            // pull-up - the chapter asks for an external one and the
            // 40 k internal one serves. The RX pad is not claimed at all;
            // it "is no longer used".
            TxPin::function(pins.tx.function,
                            {.pull = PinPull::up, .open_drain = true});
        } else {
            TxPin::function(pins.tx.function);
            RxPin::function(pins.rx.function, {.pull = PinPull::up});
        }
        if constexpr (opts.driver_enable) {
            Pin<opts.de_pin.port, opts.de_pin.pin>::function(opts.de_pin.function);
        }
        if constexpr (opts.rts) {
            Pin<opts.rts_pin.port, opts.rts_pin.pin>::function(opts.rts_pin.function);
        }
        if constexpr (opts.cts) {
            Pin<opts.cts_pin.port, opts.cts_pin.pin>::function(
                opts.cts_pin.function, {.pull = PinPull::up});
        }

        // The receiver time-out's ENABLE is UE-protected (33.8.3); its
        // value is not, and goes in with it.
        if constexpr (rx_paced && rx_tail == UsartFlag::rtof) {
            (void)S::receiver_timeout_enable(true);
            (void)S::receiver_timeout(opts.rx_timeout);
        }

        // CR3's two request bits carry no UE clause (33.8.4); they go in
        // before the enable all the same, so the first TXE and the first
        // RXNE already find their requests armed.
        if constexpr (has_tx_engine) {
            S::dma_transmit(true);
        }
        if constexpr (has_rx_engine) {
            S::dma_receive(true);
        }

        S::enable(true);
        S::clear_flags(USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF);

        if constexpr (has_rx_engine) {
            // NOT S::rxne_interrupt(true): the channel consumes RXNE -
            // into the whole receive ring, lap after lap, from here on -
            // at the options' level, very_high unless the program says
            // otherwise (UartOptions::rx_priority).
            RxEngine::arm(S::rx_data_address(), S::dma_rx_request(), RxRing::storage,
                          opts.rx_priority);
            m_rx_drained = true;
            (void)RxEngine::start();
            // The vector serves the line's IDLE edge and every receive
            // error as it happens: EIE raises FE, NE and ORE while DMAR is
            // set (33.8.4), PEIE raises PE (33.8.1). DDRE stays clear, so
            // a character with an error is not transferred and the next
            // good one is (33.8.4) - and each error is ONE ENTRY, so the
            // counters are exact rather than the once-a-look of a poll.
            S::clear_flags(UsartClear::idle);
            S::interrupts(UsartInterrupt::idle | UsartInterrupt::parity, true);
            S::error_interrupt(true);
            if constexpr (rx_kick) {
                // The wedge's watch: RXFT at seven eighths enters the
                // vector one entry and the shift register short of an
                // overrun, where the kick still loses nothing.
                S::rx_threshold_interrupt(true);
            }
        } else if constexpr (rx_paced) {
            S::clear_flags(rx_tail);
            S::rx_threshold_interrupt(true);
            S::interrupts(rx_tail == UsartFlag::rtof ? UsartInterrupt::receiver_timeout
                                                     : UsartInterrupt::idle,
                          true);
        } else {
            S::rxne_interrupt(true);
        }
        if constexpr (has_tx_engine) {
            // HIGH, a level below the receive ring: a transmit channel
            // that waits leaves TXE standing and the line idle a while,
            // and loses nothing.
            TxEngine::arm(S::tx_data_address(), S::dma_tx_request(), DmaPriority::high);
        }
        // TXE is armed on demand by write_byte() when there is no engine.

        Nvic::enable(S::irq());
        return true;
    }

    /// The ISR body of WHICHEVER DMA channel this transport owns -
    /// call it from the vector(s) the engines' channels report on.
    ///
    ///     extern "C" void DMA1_Channel2_3_IRQHandler() {
    ///         if (Serial::dma_isr()) { brio::post<SerialLines>(brio::RxActivity{}); }
    ///     }
    ///
    /// Each engine's channel reads ONLY its own flags (there is no
    /// "which channel interrupted" register on this controller), so this is
    /// safe to call from a shared vector that also serves other people's
    /// channels.
    ///
    /// On the transmit channel a completion means the block has left the
    /// ring, so exactly that many bytes are released and the next
    /// contiguous run started. On the receive channel the HALF and the
    /// FULL marks of the ring's lap arrive here (stm32g0/dma.hpp's
    /// circular arm): the full one is a lap, counted - the channel reloaded
    /// itself and nothing is re-armed - and either one hands the edge to
    /// harvest(), so a stream that never pauses for the line's IDLE still
    /// tells its consumer twice a lap.
    ///
    /// Returns harvest()'s edge: true when the receive ring holds bytes
    /// the consumer has not been told of - the same answer isr() gives,
    /// so the same kernel glue posts RxActivity on it. False on a vector
    /// entered for the transmit channel alone.
    [[gnu::always_inline]] static bool dma_isr() {
        bool edge = false;
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service();
            if ((f & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                bump(m_dma_faults);
            } else if ((f & TxEngine::flag_complete) != 0u) {
                m_tx.consume(static_cast<typename decltype(m_tx)::index_t>(
                    TxEngine::complete()));
                pump_tx();
            }
        }
        if constexpr (has_rx_engine) {
            const uint8_t f = RxEngine::service();
            if (f != 0u) {
                if ((f & RxEngine::flag_error) != 0u) {
                    (void)RxEngine::abandon();   // harvest() starts it again
                    bump(m_dma_faults);
                } else if ((f & RxEngine::flag_complete) != 0u) {
                    (void)RxEngine::complete();   // a lap: counted, nothing re-armed
                }
                edge = harvest();
            }
        }
        return edge;
    }

    /// The receive engine's housekeeping and its edge, the body both of
    /// this transport's vectors run on a receive event: isr() on the
    /// line's IDLE and on a receive error, dma_isr() on the ring's half
    /// and full marks. A public verb still, and IDEMPOTENT: an owner that
    /// calls it as well is told nothing twice.
    ///
    /// What it does: the receive error flags read and cleared through ICR
    /// - no data-register read, so no byte is taken from under the
    /// channel (33.8.11: every flag of this block clears by its ICR bit) -
    /// and counted; a channel a transfer error stopped started again, the
    /// ring cleared with it - its unread bytes gone with the abandoned
    /// lap, which dma_faults() counted; and the edge.
    ///
    /// Returns true when the ring holds bytes and the consumer has looked
    /// and found it EMPTY since the last true - the same edge contract
    /// isr() has: every byte that lands after the consumer's last look at
    /// an empty ring is reported by the next vector, and a consumer that
    /// has not drained yet is not told twice. The look is the ring's
    /// waiting(), which writes nothing, so a vector asks it while the
    /// consumer holds a run. False, and free, without an engine.
    ///
    /// ONE RACE IS NAMED AND LEFT: the restart after a transfer error
    /// clears the ring's view, which is not concurrent - a consumer holding
    /// a run across it releases into the restarted ring. A transfer error
    /// is a bus error on an address the binding fixed (10.4.7), never a
    /// receive condition, and dma_faults() reports the lap it cost.
    static bool harvest() {
        if constexpr (!has_rx_engine) {
            return false;
        } else {
            const uint32_t st = S::status();
            const uint32_t errors = st & UsartFlag::receive_errors;
            if (errors != 0u) {
                S::clear_flags(errors);   // the ICR bits sit at the ISR positions
                if ((errors & UsartFlag::ore) != 0u) {
                    bump(m_hw_overruns);
                }
                if ((errors & UsartFlag::fe) != 0u) {
                    bump(m_frame_errors);
                }
                if ((errors & UsartFlag::pe) != 0u) {
                    bump(m_parity_errors);
                }
                if ((errors & UsartFlag::ne) != 0u) {
                    bump(m_noise_errors);
                }
                if ((errors & (UsartFlag::ore | UsartFlag::fe | UsartFlag::pe)) != 0u) {
                    m_rx_lost = m_rx_lost + 1u;   // characters the channel never took
                }
            }

            // THE SILICON IS ASKED, NOT THE ARITHMETIC: a ring never stops
            // on its own, so a channel that is not running was stopped by a
            // transfer error (10.4.7) - started again from the storage's
            // first element, the view's positions with it, and the bytes
            // unread with the abandoned lap a gap in the stream.
            if (RxEngine::idle()) {
                m_rx.clear();
                m_rx_lost = m_rx_lost + 1u;
                m_rx_drained = true;
                (void)RxEngine::start();
            }
            if (!m_rx_drained || m_rx.waiting() == 0u) {
                return false;
            }
            m_rx_drained = false;
            return true;
        }
    }

    /// The core clock changed (DynamicClock fan-out): keep the
    /// same bit rate at the new rate. Called BEFORE the clock changes,
    /// so the drain runs at the rate the queued bytes were meant for;
    /// TC answers "the shifter is empty" exactly. BRR is UE-protected,
    /// so the instance is stopped around the write. Main context only.
    ///
    /// A KERNEL CLOCK THAT IS NOT PCLK OR SYSCLK DOES NOT FOLLOW, and
    /// this verb then rewrites NOTHING - which is exactly what an
    /// application asks for when it puts a console on HSI16 or LSE.
    static void rebase(uint32_t hz) {
        if constexpr (!follows_sysclk) {
            (void)hz;
        } else {
            constexpr uint32_t ring_drain_spins = 8'000'000u;
            uint32_t spins = ring_drain_spins;
            while (!m_tx.empty() && spins-- != 0u) {
            }
            spins = tail_spins;
            while ((S::status() & UsartFlag::tc) == 0u && spins-- != 0u) {
            }
            const Divisor reg = plain ? Divisor{usart_brr(hz, m_baud)}
                                      : divisor_for(baud_clock(hz), m_baud);
            if (!reg) {
                return;
            }
            S::enable(false);
            S::regs().BRR = *reg;
            S::enable(true);
        }
    }

    /// Move the LINK to a different bit rate, the clock staying put
    /// - the mirror of rebase(), which moves the clock and keeps the rate.
    ///
    /// `hz` is the SYSCLK/PCLK rate, the same argument rebase() takes;
    /// what the divisor divides is derived from it through the options'
    /// kernel clock and prescaler, so a caller never has to know which.
    ///
    /// The queued bytes are drained at the OLD rate first (they were meant
    /// for it), then BRR is written with the instance stopped, since it is
    /// UE-protected like every other frame field. False, and nothing
    /// written, when the new rate is unreachable at this clock.
    ///
    /// Main context only, and the caller owns the agreement with whatever
    /// is on the other end: a receiver still at the old rate reads noise.
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const Divisor reg = plain ? Divisor{usart_brr(hz, baud)}
                                  : divisor_for(baud_clock(hz), baud);
        if (!reg) {
            return false;
        }
        constexpr uint32_t ring_drain_spins = 8'000'000u;
        uint32_t spins = ring_drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = tail_spins;
        while ((S::status() & UsartFlag::tc) == 0u && spins-- != 0u) {
        }
        S::enable(false);
        S::regs().BRR = *reg;
        S::enable(true);
        m_baud = baud;
        return true;
    }

    static constexpr uint32_t min_hz_for(uint32_t baud) {
        if constexpr (opts.over8) {
            return S::min_hz_for_over8(baud);
        } else if constexpr (S::is_lpuart) {
            return S::min_hz_for(baud);
        } else {
            return usart_min_hz(baud);
        }
    }
    static constexpr bool can_baud(uint32_t hz, uint32_t baud) {
        if constexpr (plain) {
            return usart_brr(hz, baud).has_value();
        } else {
            return divisor_for(baud_clock(hz), baud).has_value();
        }
    }
    static uint32_t actual_baud(uint32_t hz) {
        if constexpr (plain) {
            return usart_actual_baud(hz, S::brr());
        } else if constexpr (opts.over8) {
            return S::baud_for_over8(baud_clock(hz), S::brr());
        } else {
            return S::baud_for(baud_clock(hz), S::brr());
        }
    }

    /// The instance's ONE interrupt body - call from the vector
    /// the device gives this instance (shared with others on this family).
    ///
    /// Returns true when the RX ring holds bytes the consumer has not
    /// been told of: the edge signal for kernel glue ("post RxActivity on
    /// true"). Without an engine that is the ring's empty -> non-empty
    /// transition, read across the whole entry; the consumer only empties
    /// the ring by draining it, so no wakeup is ever lost. With a receive
    /// engine no received byte passes through here: the entry is the
    /// line's IDLE or a receive error, and the edge is harvest()'s.
    [[gnu::always_inline]] static bool isr() {
        USART_TypeDef& r = S::regs();
        const uint32_t st = r.ISR;
        bool edge = false;

        if constexpr (opts.wake_from_stop != UsartWakeSource::none) {
            // WUF is a level until ICR clears it, and it is set whether
            // the core was asleep or not (33.5.21): counting it here is
            // the only place that can tell a wake from a plain byte.
            if ((st & UsartFlag::wuf) != 0u) {
                r.ICR = UsartClear::wuf;
                bump(m_wakes);
            }
        }

        if constexpr (has_rx_engine) {
            // THE CHANNEL OWNS RXNE: it reads RDR into the ring the
            // consumer reads, and this body never touches RDR - a read
            // here would take a byte from under the channel. IDLE is
            // cleared through ICR; the errors are harvest()'s, through ICR
            // too.
            if constexpr (rx_kick) {
                // THE KICK, before harvest() clears the errors: an entry for
                // the line's IDLE, a receive error or the threshold that
                // finds the FIFO STILL holding a character - a fresh read,
                // a healthy channel having emptied it within a few bus
                // cycles - is a request the channel is not serving, and
                // DMAR dropped and raised presents it anew (the class
                // comment). A threshold entry is a level: it re-enters while
                // a starved channel leaves the FIFO above it.
                if ((st & (UsartFlag::idle | UsartFlag::receive_errors | UsartFlag::rxft)) != 0u &&
                    (r.ISR & UsartFlag::rxne) != 0u) {
                    S::rx_request_restart();
                    bump(m_rx_kicks);
                }
            }
            if ((st & (UsartFlag::idle | UsartFlag::receive_errors)) != 0u) {
                r.ICR = UsartClear::idle;
                edge = harvest();
            }
        } else if constexpr (fifo_mode) {
            if constexpr (rx_paced) {
                // The tail's flag, cleared BEFORE the drain: a character
                // landing behind the drain's last look starts a time-out
                // (or an idle frame) of its own.
                if ((st & rx_tail) != 0u) {
                    r.ICR = rx_tail;
                }
            }
            // An entry for the transmitter alone touches no ring.
            if ((st & UsartFlag::rxne) != 0u) {
                edge = receive_fifo(r, st);
            }
        } else if ((st & UsartFlag::rxne) != 0u) {
            const uint8_t b = static_cast<uint8_t>(r.RDR);   // clears RXNE
            const uint32_t err = st & (UsartFlag::fe | UsartFlag::ne | UsartFlag::pe);
            if (err != 0u) {
                r.ICR = err;   // the ICR bits sit at the ISR positions
                count_errors(err);
            }
            if ((err & (UsartFlag::fe | UsartFlag::pe)) == 0u) {
                const bool was_empty = m_rx.empty();
                if (m_rx.push(b)) {
                    edge = was_empty;
                } else {
                    bump(m_rx_overruns);
                    m_rx.lost();
                }
            } else {
                m_rx.lost();
            }
        }
        if constexpr (!has_rx_engine) {
            if ((st & UsartFlag::ore) != 0u) {
                r.ICR = USART_ICR_ORECF;   // or this handler re-enters for ever
                bump(m_hw_overruns);
                // Characters lost in the silicon (33.5.4): wherever in this
                // entry's drain they fell, the consumer's next look skips
                // past all of it.
                m_rx.lost();
            }
        }

        if constexpr (has_tx_engine) {
            // The channel owns the transmitter; nothing of it is armed here.
        } else if constexpr (fifo_mode) {
            // Fill while there is room, on whichever condition the
            // options armed: TXFT at the transmit threshold (by default
            // the FIFO run empty), TXFNF under `none`. Either way the
            // loop stops at TXFNF clear, so the FIFO is filled and not
            // overrun - and an entry for the RECEIVER tops it up too.
            //
            // THE RING IS READ AS A RUN: read_span() hands over the
            // contiguous characters, the loop stores them while TXFNF
            // stands - one ISR load, one byte load and one TDR store a
            // character - and consume() releases what went, once. A ring
            // found empty disarms at once, the one the FIFO has just
            // taken the last of included, so a print does not end on an
            // entry that finds nothing to send.
            if (tx_armed()) {
                for (;;) {
                    const auto run = m_tx.read_span();
                    if (run.empty()) {
                        disarm_tx();
                        break;
                    }
                    uint32_t sent = 0;
                    while (sent < run.size() && (r.ISR & UsartFlag::txfnf) != 0u) {
                        r.TDR = run[sent];
                        ++sent;
                    }
                    m_tx.consume(static_cast<typename decltype(m_tx)::index_t>(sent));
                    if (sent < run.size()) {
                        break;   // the FIFO is full; the ring keeps the rest
                    }
                }
            }
        } else if ((st & UsartFlag::txe) != 0u && (r.CR1 & USART_CR1_TXEIE_TXFNFIE) != 0u) {
            const auto v = m_tx.pop();
            if (v) {
                r.TDR = *v;
            } else {
                r.CR1 = r.CR1 & ~USART_CR1_TXEIE_TXFNFIE;   // ring dry: TXE would re-fire
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
     * slots existed.
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
        } else if constexpr (tx_on_threshold) {
            S::tx_threshold_interrupt(true);
        } else {
            S::txe_interrupt(true);
        }
        return true;
    }

    /// Fetch one received byte; false when nothing is pending - or, with
    /// a receive engine, when the byte was written over while it was
    /// being read (the ring counts it in rx_overruns()).
    static bool read_byte(uint8_t& b) {
        if constexpr (has_rx_engine) {
            m_rx_drained = true;   // before the look (m_rx_drained says why)
        }
        const auto v = m_rx.pop();
        if (!v) {
            return false;
        }
        if constexpr (has_rx_engine) {
            m_rx_drained = false;
        }
        b = *v;
        return true;
    }

    /// Queue a run of bytes through the ring's contiguous span
    /// (util/stream.hpp's BulkSink). Returns the number queued.
    ///
    /// THE FIRST BYTE GOES BEFORE THE COPY, on the plain transport: it is
    /// pushed and the transmit interrupt armed exactly as write_byte()
    /// does, so an idle transmitter starts on it while the rest is
    /// copied. The handler disarms when it finds the ring empty - in the
    /// same entry that hands the FIFO that byte, and on the TXE entry
    /// after it without the FIFO - and the first byte was the whole ring,
    /// so the rest, once published, is armed again: the same masked arm,
    /// idempotent, two a run at most. A run that finds the ring full is
    /// write_byte()'s refusal. WITH AN ENGINE the run is copied whole and
    /// the engine pumped once, a block being what it starts on.
    static uint32_t write_bulk(std::span<const uint8_t> src) {
        uint32_t queued = 0;
        if constexpr (!has_tx_engine) {
            // write_byte()'s two steps, spelled here so the run's path
            // holds no call: a full ring is its refusal, nothing to arm.
            if (src.empty() || !m_tx.push(src[0])) {
                return 0;
            }
            if constexpr (tx_on_threshold) {
                S::tx_threshold_interrupt(true);
            } else {
                S::txe_interrupt(true);
            }
            queued = 1;
        }
        while (queued < src.size()) {
            auto dst = m_tx.write_span();
            if (dst.empty()) {
                break;
            }
            const uint32_t chunk = dst.size() < src.size() - queued
                                       ? static_cast<uint32_t>(dst.size())
                                       : static_cast<uint32_t>(src.size() - queued);
            copy_run(dst.data(), src.data() + queued, chunk);
            m_tx.publish(static_cast<typename decltype(m_tx)::index_t>(chunk));
            queued += chunk;
        }
        if constexpr (has_tx_engine) {
            pump_tx();   // ONE nudge for the whole run
        } else if (queued > 1u) {
            // The rest, behind a first byte the handler may have disarmed on.
            if constexpr (tx_on_threshold) {
                S::tx_threshold_interrupt(true);
            } else {
                S::txe_interrupt(true);
            }
        }
        return queued;
    }

    /// Drain received bytes into `dst`; returns the number copied. With a
    /// receive engine a run written over while it was being copied is
    /// not counted - its bytes are not the stream - and the copy goes on
    /// from where the ring skipped to.
    static uint32_t read_bulk(std::span<uint8_t> dst) {
        uint32_t got = 0;
        while (got < dst.size()) {
            if constexpr (has_rx_engine) {
                m_rx_drained = true;   // before the look (m_rx_drained says why)
            }
            auto src = m_rx.read_span();
            if (src.empty()) {
                break;
            }
            if constexpr (has_rx_engine) {
                m_rx_drained = false;
            }
            const uint32_t chunk = src.size() < dst.size() - got
                                       ? static_cast<uint32_t>(src.size())
                                       : static_cast<uint32_t>(dst.size() - got);
            for (uint32_t i = 0; i < chunk; ++i) {
                dst[got + i] = src[i];
            }
            if constexpr (has_rx_engine) {
                if (!m_rx.consume(chunk)) {
                    continue;
                }
            } else {
                m_rx.consume(static_cast<typename decltype(m_rx)::index_t>(chunk));
            }
            got += chunk;
        }
        return got;
    }

    /// The received bytes IN PLACE: the contiguous run ready to be read,
    /// never wrapping - the receive ring's consumer half under the ring's
    /// own names. With consume() it is util/stream.hpp's SpanSource,
    /// which SerialPort drains a run at a time with no copy at all.
    ///
    /// With a receive engine the run is the CHANNEL'S ring, read where it
    /// writes: the bytes in it are the stream as long as the consumer is
    /// less than a lap behind, and consume() says whether they were.
    static std::span<const uint8_t> read_span() {
        if constexpr (has_rx_engine) {
            m_rx_drained = true;   // before the look (m_rx_drained says why)
            const std::span<const uint8_t> run = m_rx.read_span();
            if (!run.empty()) {
                m_rx_drained = false;
            }
            return run;
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

    static auto rx_pending() { return m_rx.count(); }

    /// Every character the line carried that the receive ring will not
    /// deliver, since the program started, NEVER CLEARED (modulo 2^32) -
    /// util/stream.hpp's SkippingSource, the epoch util/serial_port.hpp
    /// compares at every run. Without an engine: the SkipRing's skips, one
    /// at the consumer's look after a character a full ring refused or
    /// FE/PE dropped, or an overrun - the ring discarding what it held. With
    /// one: the ring's skips (a lap
    /// missed, a held run refused) plus m_rx_lost - an ORE, an FE or a PE
    /// the channel never took, a restart after a transfer error - counted
    /// when the vector sees them, so the line torn is the one begun when
    /// the consumer next looks, which is the one the error cut when the
    /// consumer keeps up with its edges.
    static uint32_t rx_skips() {
        if constexpr (has_rx_engine) {
            return m_rx.skips() + m_rx_lost;
        } else {
            return m_rx.skips();
        }
    }

    /// THE WIRE IS IDLE: nothing queued, no block in flight, and TC - the
    /// last frame's stop bit off the pad, the TXFIFO empty with it in FIFO
    /// mode (33.5.5, step 8). TC cannot answer from a frame before the
    /// current one: the handler's TDR store clears it (33.8.10), and a
    /// transmit block clears it through ICR when it starts (pump_tx(),
    /// 33.5.19's step 6), so a TC standing from the previous block is
    /// gone before the block's first frame.
    static bool tx_idle() {
        if constexpr (has_tx_engine) {
            return m_tx.empty() && !TxEngine::busy() && (S::status() & UsartFlag::tc) != 0u;
        } else {
            return m_tx.empty() && (S::status() & UsartFlag::tc) != 0u;
        }
    }

    /// The receive ring's losses to a consumer that did not keep up:
    /// without an engine, each byte a full ring refused; with one, each
    /// lap of the channel's ring written over unread bytes and each run
    /// written over while it was held - a skip apiece (util/ring.hpp's
    /// HardwareRing::overruns()).
    static uint8_t rx_overruns() {
        if constexpr (has_rx_engine) {
            const uint32_t n = m_rx.overruns();
            return n > 0xFFu ? uint8_t{0xFF} : static_cast<uint8_t>(n);   // saturating
        } else {
            return m_rx_overruns;
        }
    }
    static uint8_t frame_errors() { return m_frame_errors; }
    static uint8_t parity_errors() { return m_parity_errors; }
    static uint8_t noise_errors() { return m_noise_errors; }
    static uint8_t hw_overruns() { return m_hw_overruns; }

    /// How many times the receive engine's vector presented the receive
    /// request anew - the FIFO wedge's cure fired (isr(), "THE FIFO STAYS
    /// ON UNDER A RECEIVE ENGINE"). A healthy stream never moves it; a
    /// channel starved past a character time may. Always 0, and free,
    /// without a receive engine or without the FIFO.
    static uint8_t rx_kicks() {
        if constexpr (rx_kick) {
            return m_rx_kicks;
        } else {
            return 0;
        }
    }

    /// How many times WUF was seen in the handler - the only way to tell
    /// "the USART brought the core out of Stop" from "something else
    /// did and a byte happened to be there". Always 0, and free, without
    /// the wake option.
    static uint8_t wakes() {
        if constexpr (opts.wake_from_stop != UsartWakeSource::none) {
            return m_wakes;
        } else {
            return 0;
        }
    }

    /// DMA blocks this transport threw away because the controller had
    /// stopped running them (a transfer error disables a channel in
    /// hardware, RM0444 10.4.7). Always 0, and free, without an engine.
    static uint8_t dma_faults() {
        if constexpr (has_tx_engine || has_rx_engine) {
            return m_dma_faults;
        } else {
            return 0;
        }
    }

    static void clear_errors() {
        if constexpr (has_rx_engine) {
            m_rx.clear_overruns();
        } else {
            m_rx_overruns = 0;
        }
        m_frame_errors = 0;
        m_parity_errors = 0;
        m_noise_errors = 0;
        m_hw_overruns = 0;
        if constexpr (rx_kick) {
            m_rx_kicks = 0;
        }
        if constexpr (opts.wake_from_stop != UsartWakeSource::none) {
            m_wakes = 0;
        }
    }

    /// Put everything back: interrupts off, instance disabled and reset,
    /// pads to analog, bus clock off. The rings are dropped.
    ///
    /// THE INSTANCE STOPS BEFORE ITS CHANNELS: a receive ring never ends,
    /// and 10.4.5 stops a circular channel by stopping the peripheral's
    /// requests first and disabling the channel after.
    static void release() {
        Nvic::disable(S::irq());
        S::enable(false);
        if constexpr (has_tx_engine) {
            TxEngine::stop();
        }
        if constexpr (has_rx_engine) {
            RxEngine::stop();
        }
        S::reset();
        TxPin::release();
        if constexpr (!opts.half_duplex) {
            RxPin::release();
        }
        if constexpr (opts.driver_enable) {
            Pin<opts.de_pin.port, opts.de_pin.pin>::release();
        }
        if constexpr (opts.rts) {
            Pin<opts.rts_pin.port, opts.rts_pin.pin>::release();
        }
        if constexpr (opts.cts) {
            Pin<opts.cts_pin.port, opts.cts_pin.pin>::release();
        }
        if constexpr (opts.wake_from_stop != UsartWakeSource::none) {
            (void)S::wake_line(false);
        }
        S::bus_clock(false);
        m_rx.clear();
        m_tx.clear();
    }

private:
    static inline volatile uint8_t m_wakes = 0;

    /// `Divisor{...}` around the plain arm is the C++ conditional
    /// operator's doing and not a conversion anyone wants: for a USART
    /// the two arms are already the same type and the braces are a copy
    /// the compiler removes; for an LPUART, where `plain` is a constant
    /// false, they are what stops optional<uint16_t> and
    /// optional<uint32_t> from being an AMBIGUOUS composite type (each
    /// converts to the other, so the operator has no answer).
    ///
    /// THE DEFAULT ARRANGEMENT, spelled out so that every
    /// baud-arithmetic verb below can name one and the same expression
    /// for it CHARACTER FOR CHARACTER. Folding `hz /
    /// usart_prescaler_divisor(div1)` to `hz` gives the same value and
    /// NOT the same code: an image moves by forty bytes. Byte identity
    /// outranks API economy, and a `plain` branch costs nothing.
    static constexpr bool plain = !S::is_lpuart &&
                                  opts.kernel_clock == UsartClock::pclk &&
                                  opts.prescaler == UsartPrescaler::div1 &&
                                  !opts.over8;
    /// Whether a SYSCLK change moves this instance's divisor at all.
    static constexpr bool follows_sysclk = opts.kernel_clock == UsartClock::pclk ||
                                           opts.kernel_clock == UsartClock::sysclk;

    /// The divisor this instantiation's oversampling asks for. The
    /// dispatch is `if constexpr` and the two arithmetics are SIBLING
    /// verbs of the resource - not one verb with a bool - because a
    /// defaulted third argument on usart_brr() moved a pre-existing
    /// image by forty bytes with the folded code identical.
    static constexpr Divisor divisor_for(uint32_t ker_hz, uint32_t baud) {
        if constexpr (opts.over8) {
            return S::brr_for_over8(ker_hz, baud);
        } else {
            return S::brr_for(ker_hz, baud);
        }
    }

    /// The SYSCLK/PCLK rate a caller hands in, turned into what the
    /// divisor divides. Folds to the argument itself for the default
    /// options, which is what keeps set_baud/can_baud/actual_baud
    /// byte-identical.
    static constexpr uint32_t baud_clock(uint32_t hz) {
        if constexpr (opts.kernel_clock == UsartClock::pclk ||
                      opts.kernel_clock == UsartClock::sysclk) {
            return usart_kernel_hz(hz, opts.prescaler);
        } else {
            (void)hz;
            return usart_kernel_hz(opts.kernel_clock == UsartClock::hsi16
                                       ? 16'000'000u : 32768u,
                                   opts.prescaler);
        }
    }

    /// Which transmit condition this transport rides, and how it is
    /// armed and disarmed. Without the FIFO it is TXE (TXEIE); with it,
    /// TXFT (TXFTIE) at the transmit threshold - by default the FIFO run
    /// empty - and TXFNF where the threshold is `none`.
    static constexpr bool tx_on_threshold =
        fifo_mode && opts.tx_threshold != UartFifoThreshold::none;

    /// The bound of the TC wait rebase() and set_baud() make before a
    /// BRR write, in spins - a guard against a wedge, not a time. TC
    /// rises when the last frame has left the shift register AND, in FIFO
    /// mode, the TXFIFO is empty (33.5.5, step 8), so what is still to go
    /// once the ring is empty is two frames without the FIFO (the data
    /// register and the shift register) and up to nine with it (eight
    /// places and the shift register): the FIFO's depth scales the bound,
    /// so the margin the one-register transport had over its tail is the
    /// margin the FIFO's tail gets.
    static constexpr uint32_t tail_spins =
        fifo_mode ? 200'000u * (uint32_t{S::fifo_depth} + 1u) : 200'000u;

    /// write_bulk()'s copy into the ring's free run. THE RUNTIME'S memcpy
    /// WHERE ITS FIXED COST PAYS, the byte loop below it - measured with
    /// bench_stm32's letter u on the Cortex-M0+ at two wait states: the
    /// loop is 4 cycles and 9 a byte, memcpy 70 and under one a byte when
    /// the two ends share their word alignment (its LDM/STM block, rt.cpp)
    /// and its own byte loop otherwise, never ahead. So memcpy takes a run
    /// of `copy_crossover` bytes or more whose ends are aligned alike - the
    /// two cost the same near 13 - and the loop takes the rest: two
    /// pointers and no index, the test at the bottom, a load, a store, two
    /// steps and one branch a byte. `n` is at least one.
    static constexpr uint32_t copy_crossover = 16;
    [[gnu::always_inline]] static void copy_run(uint8_t* to, const uint8_t* from, uint32_t n) {
        if (n >= copy_crossover &&
            ((reinterpret_cast<uintptr_t>(to) ^ reinterpret_cast<uintptr_t>(from)) & 3u) == 0u) {
            std::memcpy(to, from, n);
            return;
        }
        uint8_t* const end = to + n;
        do {
            *to++ = *from++;
        } while (to != end);
    }

    /// The three per-character error flags counted (the caller cleared
    /// them): FE and PE drop the character, NE keeps it.
    [[gnu::always_inline]] static void count_errors(uint32_t err) {
        if ((err & UsartFlag::fe) != 0u) {
            bump(m_frame_errors);
        }
        if ((err & UsartFlag::pe) != 0u) {
            bump(m_parity_errors);
        }
        if ((err & UsartFlag::ne) != 0u) {
            bump(m_noise_errors);
        }
    }

    /// Drain the RXFIFO into the ring, from the entry's own ISR read `s`
    /// on: per character one ISR load and its test, one RDR load and the
    /// ring's push, inline - the error flags tested on the ISR word
    /// already in hand and served off the character's path. The ring's
    /// edge is read across the whole entry, empty at its start and not at
    /// its end, since nothing consumes while a handler runs.
    ///
    /// 33.5.4: the RXFIFO carries PE/NE/FE WITH EACH ENTRY and ISR reports
    /// the flags of the entry AT THE HEAD - so ISR is re-read before every
    /// RDR, or the attribution slides by one and a good byte inherits its
    /// neighbour's framing error.
    ///
    /// A character the ring refuses or FE or PE drops is reported to the
    /// ring where it is met, on its rare path: where in the stream it fell
    /// is nobody's business, since the consumer's look after it skips
    /// everything queued (util/ring.hpp's SkipRing), and nothing is kept
    /// across the loop for it.
    [[gnu::always_inline]] static bool receive_fifo(USART_TypeDef& r, uint32_t s) {
        const bool was_empty = m_rx.empty();
        for (;; s = r.ISR) {
            if ((s & UsartFlag::rxne) == 0u) {
                break;
            }
            take(r, s);
        }
        return was_empty && !m_rx.empty();
    }

    /// The character at the RXFIFO's head, `s` the ISR word read for it.
    [[gnu::always_inline]] static void take(USART_TypeDef& r, uint32_t s) {
        const uint32_t err = s & (UsartFlag::fe | UsartFlag::ne | UsartFlag::pe);
        if (err != 0u) [[unlikely]] {
            const uint8_t b = static_cast<uint8_t>(r.RDR);
            r.ICR = err;   // after the RDR read: the flags are this entry's
            count_errors(err);
            if ((err & (UsartFlag::fe | UsartFlag::pe)) == 0u) {
                keep(b);
            } else {
                m_rx.lost();
            }
            return;
        }
        keep(static_cast<uint8_t>(r.RDR));
    }

    /// One received character into the ring, or counted as the ring's
    /// overrun and reported lost.
    [[gnu::always_inline]] static void keep(uint8_t b) {
        if (!m_rx.push(b)) [[unlikely]] {
            bump(m_rx_overruns);
            m_rx.lost();
        }
    }

    static void disarm_tx() {
        if constexpr (tx_on_threshold) {
            S::tx_threshold_interrupt(false);
        } else {
            S::txe_interrupt(false);
        }
    }
    static bool tx_armed() {
        if constexpr (tx_on_threshold) {
            return (S::regs().CR3 & USART_CR3_TXFTIE) != 0u;
        } else {
            return S::txe_interrupt();
        }
    }

    /**
     * Hand the transmit engine the ring's next contiguous run, if it is
     * free to take one.
     *
     * THE MASK COVERS THE CLAIM AND NOTHING ELSE. Two contexts start
     * blocks - the thread through write_byte()/write_bulk() and the
     * channel's completion through dma_isr() - so the engine's busy flag
     * is taken under the guard, a test-and-set of a few instructions.
     * Everything after it runs unmasked, and that is safe on this
     * controller for a reason the claim itself carries: an engine that
     * was free has no block in flight, so no completion, error or
     * consume() can arrive under the programming, and the ring's read
     * side is this call's until the block is enabled. A claim that finds
     * the ring empty (the handler's call after the last run) is given
     * back.
     *
     * NOTHING KICKS THE FIRST BEAT, AND THAT IS THIS CONTROLLER'S OWN
     * FACT. A controller that latched a trigger on the RISE would need
     * to be asked whether the peripheral's request was already standing
     * and software-triggered if it was. Here 10.4.3's handshake is
     * level-driven: the channel is enabled, it sees TXE asserted, it
     * writes TDR. Measured rather than assumed, because a wrong answer
     * is a transmitter that never starts.
     */
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            {
                typename Stm32g0Platform<>::CriticalSection cs;
                if (!TxEngine::claim()) {
                    return;
                }
            }
            auto run = m_tx.read_span();
            if constexpr (tx_size > 0xFFFFu) {
                // CNDTR counts to 65535: a ring that big hands a longer
                // run over in two blocks.
                if (run.size() > 0xFFFFu) {
                    run = run.first(0xFFFFu);
                }
            }
            if (run.empty()) {
                TxEngine::unclaim();
                return;
            }
            // 33.5.19, step 6: TC cleared before the channel starts, so
            // tx_idle() never reads the previous block's stop bit as this
            // one's - the channel's TDR stores are not the software clear.
            S::clear_flags(UsartClear::tc);
            if (!TxEngine::launch(run)) {
                TxEngine::unclaim();
            }
        }
    }

};

/// The task over a USART instance: the name every application knows.
template <uint8_t n, UartPins pins, uint32_t rx_size = 64, uint32_t tx_size = 256,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine,
          UartOptions opts = UartOptions{}>
using Uart = UartTask<Usart<n>, pins, rx_size, tx_size, TxEngine, RxEngine, opts>;

/// RS-485: a Uart with the driver enable of 33.5.20 on the RTS pad. The
/// DE timings are SAMPLE times (1/16 of a bit at OVER8 = 0), which is
/// the one thing about this feature that is easy to get wrong.
template <uint8_t n, UartPins pins, PinSel de_pin, uint8_t assertion = 0,
          uint8_t deassertion = 0, uint32_t rx_size = 64, uint32_t tx_size = 256,
          UartOptions base = UartOptions{}>
using Rs485 = UartTask<Usart<n>, pins, rx_size, tx_size, NoDmaEngine, NoDmaEngine,
                       uart_with_driver_enable(base, de_pin, assertion, deassertion)>;

// ---- the chapter's other personalities -----------------------------------------
//
// Each of these is a POLLED task over the resource, with no ring and no
// interrupt of its own: they are protocol shapes, not byte transports,
// and every one of them is used a few characters at a time. A ring-fed
// version of any of them is a Uart with the right options plus that
// shape's own configure() call, which is what the bench suite does where
// it wants both.

/**
 * SyncHost<n, pins, ck>: the synchronous master of 33.5.14 - the
 * asynchronous transmitter with a CK output beside it, so the data on TX
 * is clocked.
 *
 * WHAT IS BUILT AND WHAT IS NOT. The master's CK generation, its
 * polarity, its phase and LBCL (the clock pulse on the last data bit)
 * are all here and are all observable on the CK pad with no peer at all.
 * THE DATA PATH IS DECLINED and says so: a synchronous link needs
 * something at the other end to clock, and this desk has one board. The
 * SLAVE half (CR2.SLVEN, DIS_NSS and the underrun flag UDR) is the same
 * story one step further - the register verbs exist on the resource, the
 * task does not, and neither is claimed to work.
 */
template <uint8_t n, UartPins pins, PinSel ck>
struct SyncHost {
    using S = Usart<n>;
    static_assert(S::has_synchronous_mode,
                  "brio SyncHost: synchronous mode belongs to every USART of this "
                  "family (RM0444 table 184 gives it to the FULL and the BASIC "
                  "column) - but to no LPUART, which has no CK output at all");
    static_assert(ck.valid(), "brio SyncHost: the CK pad must be a real pin");

    SyncHost() = delete;
    using Resource = S;

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, const SyncConfig& sync,
                     const UartFormat& format = {}) {
        static_assert(clock_follows<Clock, SyncHost>(),
                      "brio SyncHost: no rebase() - a dynamic clock is refused");
        const std::optional<uint16_t> reg = usart_brr(clock_hz(clock), baud);
        if (!reg) {
            return false;
        }
        S::bus_clock(true);
        S::reset();
        (void)S::kernel_clock(UsartClock::pclk);
        if (!S::configure(format, *reg)) {
            return false;
        }
        if (!S::synchronous(sync)) {
            return false;
        }
        Pin<pins.tx.port, pins.tx.pin>::function(pins.tx.function);
        Pin<pins.rx.port, pins.rx.pin>::function(pins.rx.function,
                                                 {.pull = PinPull::up});
        Pin<ck.port, ck.pin>::function(ck.function);
        S::enable(true);
        return true;
    }

    /// One character out, polled. The CK pulses come with it.
    static bool send(uint8_t b, uint32_t spins = 200'000u) {
        while ((S::status() & UsartFlag::txe) == 0u) {
            if (spins-- == 0u) {
                return false;
            }
        }
        S::write_data(b);
        return true;
    }
    /// Wait for the transmission to be complete on the wire.
    static bool drain(uint32_t spins = 400'000u) {
        while ((S::status() & UsartFlag::tc) == 0u) {
            if (spins-- == 0u) {
                return false;
            }
        }
        return true;
    }

    static void release() {
        S::enable(false);
        S::reset();
        Pin<pins.tx.port, pins.tx.pin>::release();
        Pin<pins.rx.port, pins.rx.pin>::release();
        Pin<ck.port, ck.pin>::release();
        S::bus_clock(false);
    }
};

/**
 * IrdaLink<n, pins>: the SIR ENDEC of 33.5.18 - a 3/16-of-a-bit pulse
 * for every zero on the way out and an RZI decoder on the way in.
 *
 * The prescaler is BOTH the low-power divisor and the receive glitch
 * filter, and it may not be zero (the chapter says the encoder/decoder
 * "doesn't work when PSC = 0"); the stop bits must be one, and the
 * chapter's exclusions - LINEN, CLKEN, SCEN, HDSEL - are refused by the
 * resource rather than written over.
 *
 * IT IS HALF DUPLEX BY PHYSICS and the chapter says so: while the
 * encoder is busy the decoder ignores the line. The pads stay two -
 * IrDA_OUT is the TX pad and IrDA_IN the RX pad - because the ENDEC sits
 * between the USART and the transceiver, not between the USART and the
 * pin.
 */
template <uint8_t n, UartPins pins>
struct IrdaLink {
    using S = Usart<n>;
    static_assert(S::is_full,
                  "brio IrdaLink: the IrDA ENDEC is a FULL instance's (RM0444 table 184)");
    static_assert(uart_pins_valid(pins), "brio IrdaLink: two real, different pads");

    IrdaLink() = delete;
    using Resource = S;

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, const IrdaConfig& cfg,
                     UartParity parity = UartParity::none) {
        static_assert(clock_follows<Clock, IrdaLink>(),
                      "brio IrdaLink: no rebase() - a dynamic clock is refused");
        const std::optional<uint16_t> reg = usart_brr(clock_hz(clock), baud);
        if (!reg) {
            return false;
        }
        S::bus_clock(true);
        S::reset();
        (void)S::kernel_clock(UsartClock::pclk);
        // 33.5.18: one stop bit, and OVER8 = 0 (equation 2 of 33.5.7).
        if (!S::configure({.bits = UartBits::eight, .parity = parity, .stop_bits = 1},
                          *reg)) {
            return false;
        }
        if (!S::irda(cfg)) {
            return false;
        }
        Pin<pins.tx.port, pins.tx.pin>::function(pins.tx.function);
        Pin<pins.rx.port, pins.rx.pin>::function(pins.rx.function,
                                                 {.pull = PinPull::up});
        S::enable(true);
        return true;
    }

    static bool send(uint8_t b, uint32_t spins = 200'000u) {
        while ((S::status() & UsartFlag::txe) == 0u) {
            if (spins-- == 0u) {
                return false;
            }
        }
        S::write_data(b);
        return true;
    }
    static bool drain(uint32_t spins = 400'000u) {
        while ((S::status() & UsartFlag::tc) == 0u) {
            if (spins-- == 0u) {
                return false;
            }
        }
        return true;
    }
    /// One character in, if one has arrived. The error flags travel with
    /// it, exactly as in the non-FIFO view of the plain receiver.
    static std::optional<uint8_t> receive(uint32_t& flags) {
        const uint32_t st = S::status();
        if ((st & UsartFlag::rxne) == 0u) {
            return std::nullopt;
        }
        flags = st & UsartFlag::receive_errors;
        const uint8_t b = S::read_data();
        if (flags != 0u) {
            S::clear_flags(flags);
        }
        return b;
    }

    static void release() {
        S::enable(false);
        S::reset();
        Pin<pins.tx.port, pins.tx.pin>::release();
        Pin<pins.rx.port, pins.rx.pin>::release();
        S::bus_clock(false);
    }
};

/**
 * AutoBaud<n>: the detector of 33.5.9, as the three-step protocol it is
 * - state a non-zero BRR, arm a pattern, wait for ABRF and then read
 * what the silicon put in BRR.
 *
 * IT IS A VIEW AND NOT A TRANSPORT: it configures nothing about the
 * frame and claims no pad, because the whole point is to run it on a
 * receiver that is already up. The caller arms it, feeds the line a
 * character of the right shape, and asks.
 */
template <uint8_t n>
struct AutoBaud {
    using S = Usart<n>;
    static_assert(S::is_full,
                  "brio AutoBaud: automatic baud rate detection is a FULL instance's "
                  "(RM0444 table 184)");

    AutoBaud() = delete;
    using Resource = S;

    /// Arm a mode. The instance must be DISABLED (CR2 is UE-protected)
    /// and BRR already non-zero - 33.5.9's own precondition, which the
    /// resource enforces.
    static bool arm(AutoBaudMode m) { return S::auto_baud(m); }
    static bool disarm() { return S::auto_baud_off(); }

    /// A new measurement on an already-armed detector, with the USART
    /// running: RQR.ABRRQ.
    static void restart() { S::auto_baud_restart(); }

    static bool done() { return (S::status() & UsartFlag::abrf) != 0u; }
    static bool failed() { return (S::status() & UsartFlag::abre) != 0u; }
    static uint16_t learned_brr() { return S::brr(); }
    /// What the learned divisor is worth at the kernel rate the caller
    /// states - the only way to compare a detection with the truth.
    static uint32_t learned_baud(uint32_t ker_hz) {
        return S::oversampling() ? usart_actual_baud_over8(ker_hz, S::brr())
                                 : usart_actual_baud(ker_hz, S::brr());
    }

    /// Poll until the detector has an answer, or the spins run out.
    /// True when ABRF rose AND ABRE stayed clear.
    static bool wait(uint32_t spins = 4'000'000u) {
        while (!done()) {
            if (spins-- == 0u) {
                return false;
            }
        }
        return !failed();
    }
};

/**
 * Smartcard<n, pad>: the ISO 7816-3 personality of 33.5.17 - 8 bits plus
 * parity, 1.5 stop bits, ONE WIRE on the TX pad, a guard time counted in
 * baud periods after the stop bit, an automatic NACK on a parity error
 * and up to seven retries.
 *
 * THE PAD IS OPEN DRAIN WITH A PULL-UP, exactly as the half-duplex mode
 * asks, and for the same reason: the card drives the same wire. The
 * optional CK output is a SECOND pad and a separate claim, because a
 * card that supplies its own clock does not want one.
 *
 * WHAT A BENCH WITHOUT A CARD CAN STILL SEE, and what this task is
 * shaped to let it see: the frame on the wire (the loop-back through the
 * driver's own receiver), the guard time as a delay of TC, the CK
 * frequency on its pad, and the NACK - which is a caller pulling the
 * line low during the 1.5 stop bit, since a NACK is nothing more.
 */
template <uint8_t n, PinSel pad, PinSel ck = PinSel{}>
struct Smartcard {
    using S = Usart<n>;
    static_assert(S::is_full,
                  "brio Smartcard: smartcard mode is a FULL instance's (RM0444 table "
                  "184) - and the only one of the three personalities that is");
    static_assert(pad.valid(), "brio Smartcard: the single wire needs a real pad");

    Smartcard() = delete;
    using Resource = S;

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, const SmartcardConfig& cfg) {
        static_assert(clock_follows<Clock, Smartcard>(),
                      "brio Smartcard: no rebase() - a dynamic clock is refused");
        const std::optional<uint16_t> reg = usart_brr(clock_hz(clock), baud);
        if (!reg) {
            return false;
        }
        S::bus_clock(true);
        S::reset();
        (void)S::kernel_clock(UsartClock::pclk);
        // 33.5.17: 8 bits PLUS parity is M = 1 (nine bits including it),
        // and the parity is even.
        if (!S::configure({.bits = UartBits::nine, .parity = UartParity::even,
                           .stop_bits = 1},
                          *reg)) {
            return false;
        }
        if (!S::smartcard(cfg)) {
            return false;
        }
        // One wire, open drain, pulled up - the card and the USART share
        // it and neither may drive it high.
        Pin<pad.port, pad.pin>::function(pad.function,
                                         {.pull = PinPull::up, .open_drain = true});
        if constexpr (ck.valid()) {
            Pin<ck.port, ck.pin>::function(ck.function);
        }
        S::enable(true);
        return true;
    }

    static bool send(uint8_t b, uint32_t spins = 200'000u) {
        while ((S::status() & UsartFlag::txe) == 0u) {
            if (spins-- == 0u) {
                return false;
            }
        }
        S::write_data(b);
        return true;
    }
    /// TC, which in this mode rises only AFTER the guard time (33.5.17).
    static bool drain(uint32_t spins = 4'000'000u) {
        while ((S::status() & UsartFlag::tc) == 0u) {
            if (spins-- == 0u) {
                return false;
            }
        }
        return true;
    }
    /// TCBGT - "transmission complete BEFORE guard time", which is the
    /// flag that says the frame left and no NACK came back, without
    /// waiting for the guard time TC waits for.
    static bool complete_before_guard_time() {
        return (S::status() & UsartFlag::tcbgt) != 0u;
    }
    static std::optional<uint8_t> receive(uint32_t& flags) {
        const uint32_t st = S::status();
        if ((st & UsartFlag::rxne) == 0u) {
            return std::nullopt;
        }
        flags = st & UsartFlag::receive_errors;
        const uint8_t b = S::read_data();
        if (flags != 0u) {
            S::clear_flags(flags);
        }
        return b;
    }
    /// RTOR.BLEN's end-of-block flag (T = 1).
    static bool end_of_block() { return (S::status() & UsartFlag::eobf) != 0u; }

    static void release() {
        S::enable(false);
        S::reset();
        Pin<pad.port, pad.pin>::release();
        if constexpr (ck.valid()) {
            Pin<ck.port, ck.pin>::release();
        }
        S::bus_clock(false);
    }
};

} // namespace brio
