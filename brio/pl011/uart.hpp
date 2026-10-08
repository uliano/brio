/*
 * uart.hpp
 *
 * THE ARM PRIMECELL UART (PL011), written once for every family that
 * carries it: an IP STRATUM, a directory named for a peripheral DESIGN
 * rather than for a silicon, sitting where a core stratum sits - above
 * util/, below the families (docs/pl011/README.md, and the rule in
 * docs/design/overview.md: "A core stratum sits between util/ and the
 * families that share a core").
 *
 * The two strata every brio serial driver has (docs/design/serial.md)
 * are both here:
 *
 *  Pl011Uart<Chip, n>    the RESOURCE, named for the IP: which instance,
 *                        where its registers are, its reset and its
 *                        interrupt line (both asked of the family), the
 *                        line control, the two baud divisors and the
 *                        write that latches them, the FIFOs and their
 *                        trigger levels, the interrupt mask/status/clear
 *                        trio, the flag register;
 *  Pl011Transport<Chip, n, pins, ...>
 *                        the TASK: the asynchronous byte transport with
 *                        ring buffers and one ISR body - every console's
 *                        personality, a ByteTransport for print() and
 *                        SerialPort. Its public surface is every target's
 *                        Uart surface, which is what lets
 *                        util/serial_port.hpp and print() compile over it
 *                        untouched.
 *
 * THIS FILE KNOWS NO CHIP. It includes kernel/ and util/ and nothing
 * else: no vendor header, no family header, and it never names a reset
 * controller, a clock, a pad function, an interrupt controller or a DMA
 * request. WHAT A FAMILY OWES IT is the `Pl011Chip` concept below,
 * satisfied by one traits type the family's own header defines; that
 * header also keeps the PUBLIC NAMES apps use (`Pl011<n>`, `Uart<n,
 * pins, ...>`) as aliases of these two templates, so nothing above ever
 * spells a Chip.
 *
 * WHAT THE IP DOES, and shapes the task:
 *  - the baud rate divisor is UARTCLK / (16 x baud) as a 16-bit integer
 *    (UARTIBRD) plus a 6-bit fraction (UARTFBRD); the two registers take
 *    effect only on the NEXT write of UARTLCR_H, so the divisor is
 *    written before the line control, and a divisor change alone is
 *    followed by a dummy LCR_H write;
 *  - LCR_H, IBRD and FBRD are written with the UART DISABLED (the
 *    PL011's rule): the task disables around every configuration;
 *  - the receive FIFO stores the framing, parity and break flags WITH
 *    EACH BYTE (bits 8..10 of UARTDR), so a corrupted byte is dropped
 *    precisely; the overrun bit (bit 11) says a byte was lost AFTER the
 *    one read, and that one is good. How DEEP the two FIFOs are is a
 *    synthesis parameter and therefore the family's (`Chip::fifo_depth`);
 *  - THE TRANSMIT INTERRUPT IS AN EDGE THAT STAYS LATCHED. ARM's text,
 *    which both Raspberry Pi data sheets reproduce (RP2040 4.2.6.3,
 *    RP2350 12.1.6.3): it is asserted when the transmit FIFO is at or
 *    below its trigger level, cleared by writing the FIFO above the level
 *    or by UARTICR, and "based on a transition through a level, rather
 *    than on the level itself". Measured on the RP2350
 *    (docs/rp2350/uart.md): TXRIS is SET when the FIFO falls through the
 *    level, masked or not; it STAYS set until a write takes the FIFO above
 *    the level or UARTICR clears it; and the level alone never sets it
 *    again - a TXIM armed over a FIFO at or below its level with TXRIS
 *    clear never fires. The transmit side is built on exactly that:
 *      - a byte with nothing queued ahead of it and room in the FIFO is
 *        WRITTEN STRAIGHT INTO UARTDR by the caller, so an idle
 *        transmitter takes a FIFO's depth of bytes with no interrupt;
 *      - a byte behind a full FIFO, or behind bytes already queued, goes
 *        into the ring and ARMS TXIM. Every queued byte has an edge
 *        coming or one latched: the first queues only behind a FULL FIFO,
 *        which is above its level, and from there the FIFO draining
 *        latches TXRIS while a write either stays above the level or
 *        leaves TXRIS as it was - so the one store that could take the
 *        guarantee away is a clear of TXRIS in UARTICR, which the
 *        transmit path never writes. One interrupt per FIFO level, each
 *        refilling the FIFO from the ring, and TXIM disarmed by the
 *        handler when the ring runs dry;
 *      - a byte the full ring REFUSES writes nothing at all: the ring is
 *        not empty, so TXIM is armed and its edge is what drains the
 *        ring. A caller spinning on a full ring costs no interrupt.
 *    The ring's consumer is the handler alone, and the caller writes
 *    UARTDR only while the ring is empty: a handler runs to completion,
 *    so a ring the caller sees empty has every byte queued before it in
 *    the FIFO already, and the wire carries the bytes in the order they
 *    were written;
 *  - the receive side arms RXIM (the FIFO at its trigger level) AND
 *    RTIM (bytes waiting and the line idle for 32 bit periods), so a
 *    short burst is delivered after one character time and a long one
 *    every sixteen bytes; the handler drains the FIFO whole;
 *  - the eleven maskable sources combine into ONE interrupt line per
 *    instance, which the family names.
 *
 * THE ENGINE SLOTS (the family's DmaTxEngine / DmaRxEngine, the other
 * families' shape). With a TRANSMIT engine the handler never touches the
 * transmit FIFO: write_byte() and write_bulk() queue into the ring and
 * pump_tx() starts a block over the ring's contiguous run whenever it can
 * CLAIM the engine - the claim alone under the mask, the block programmed
 * after it; the block's completion - on the DMA line the engine reports
 * on, dma_isr() - releases exactly that run and starts the next.
 * The transmit FIFO's own data request (UARTDMACR.TXDMAE) paces it,
 * credit by credit, so no kick is ever needed. With a RECEIVE engine the
 * receive interrupts stay off and the engine fills the ring's free run
 * straight from UARTDR; a run that fills is published and re-armed by
 * its completion on the DMA line, whose dma_isr() answers the ring's
 * edge, and a run still filling is read off the engine's own count by
 * harvest(), a VERB the owner calls at its own pace, which publishes it
 * and re-arms the run. The engine moves BYTES and the per-entry error
 * flags of UARTDR are not among them - a framed or break entry's byte
 * lands in the ring like any other - so the error interrupts are armed
 * under an engine and isr() counts each received error, clearing it
 * through UARTICR and never reading UARTDR. NO VECTOR ENDS A BURST UNDER AN ENGINE: the receive time-out
 * needs a character waiting, and the channel's single request leaves
 * none (measured, docs/pl011/README.md) - the interrupt receiver is the
 * burst path with an edge. The DMA and the handler never touch one FIFO
 * at once.
 *
 * NOT built, each with its reason in docs/pl011/README.md: hardware flow
 * control, IrDA, the modem status inputs and outputs, the stick-parity
 * bit. The slots for DMA engines ARE here, empty, so the type's shape
 * does not move when a family fills them.
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <cstring>
#include <optional>
#include <span>

#include "kernel/platform.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

// ---- what a family owes this file ---------------------------------------------

/**
 * The absent engine, as the concept measures a family's
 * `engines_distinct` against it: `present` is all this file ever asks of
 * an engine slot, and a family's own empty-slot tag says the same.
 */
struct Pl011AbsentEngine {
    Pl011AbsentEngine() = delete;
    static constexpr bool present = false;
};

/**
 * THE CONTRACT: what a family's chip-traits type states so that this
 * file can drive the block without knowing which silicon carries it.
 * Every member is here because some line below needs it.
 *
 * The one thing the concept names and cannot check here is
 * `uartclk_hz(clock)` - it is a template over the family's OWN clock
 * types, of which this file knows none - so it has a concept of its own,
 * `Pl011ChipClock`, checked at the one call site that needs it, init().
 */
template <typename C>
concept Pl011Chip =
    requires {
        /// The register block of one instance, as the family's device
        /// description spells it: the member names are the PL011's own
        /// (UARTDR, UARTFR, UARTLCR_H, ...).
        typename C::Regs;
        /// What the family's interrupt controller calls a line.
        typename C::Irq;
        /// That controller: enable and disable one line. A family may
        /// carry an NVIC under one architecture and something else under
        /// another, so this file never names one.
        typename C::Interrupts;
        /// The RAII critical section of this family - what the transport
        /// takes while it shares the ring's producer side with a handler.
        typename C::Guard;
        /// The brio Platform the byte rings are built on (util/ring.hpp
        /// asks it for the atomic width and the guard).
        typename C::Platform;
        /// The family's pin-set type, one pad per direction: which pads
        /// exist and which function routes a UART to them is the
        /// family's table, never this file's.
        typename C::Pins;
        /// What the family's DMA calls a peripheral request.
        typename C::DmaRequest;
        /// How many instances this family carries, and how deep the two
        /// FIFOs were synthesized.
        { C::instances } -> std::convertible_to<uint8_t>;
        { C::fifo_depth } -> std::convertible_to<uint8_t>;
        /// The run length from which the runtime's memcpy copies a run
        /// into the transmit ring faster than a byte loop, its two ends
        /// sharing their word alignment: a fact of the CORE and the
        /// memory the code runs from, measured on each family (its bench
        /// app's letter u), never this file's.
        { C::copy_crossover } -> std::convertible_to<uint32_t>;
    } &&
    Platform<typename C::Platform> &&
    requires(volatile uint32_t& reg, uint32_t bits, typename C::Pins pins, uint8_t instance,
             typename C::Irq line) {
        /// Instance 0 answers every per-instance question; so does every
        /// other instance the family has. Each is a template on the
        /// instance number so the answer is a compile-time constant -
        /// a register address, a line, a request.
        { C::template regs<0>() } -> std::same_as<typename C::Regs&>;
        { C::template irq<0>() } -> std::same_as<typename C::Irq>;
        /// The block from its reset state, held, and whether it is out:
        /// on one family a reset controller, on another a clock gate.
        { C::template reset<0>() } -> std::same_as<bool>;
        { C::template released<0>() } -> std::same_as<bool>;
        C::template hold<0>();
        /// The two request numbers a transport hands its engines.
        { C::template tx_request<0>() } -> std::same_as<typename C::DmaRequest>;
        { C::template rx_request<0>() } -> std::same_as<typename C::DmaRequest>;
        /// One bus write that sets or clears bits of a register, with no
        /// read-modify-write: a handler and the loop each touch their
        /// own bits of UARTIMSC, and the transport's enable() leaves the
        /// rest of UARTCR alone.
        C::set_bits(reg, bits);
        C::clear_bits(reg, bits);
        /// The pads: is this pin set legal for this instance, and which
        /// pad carries each direction. Then the PAD ITSELF as a type,
        /// with the function code that routes a UART to it and the
        /// electrical setup each direction wants - the two verbs this
        /// file calls on a pad are handing it over and taking it back.
        { C::pins_valid(instance, pins) } -> std::same_as<bool>;
        { C::tx_pad(pins) } -> std::same_as<uint8_t>;
        { C::rx_pad(pins) } -> std::same_as<uint8_t>;
        C::template Pad<0>::function(C::pad_function, C::tx_pad_config());
        C::template Pad<0>::function(C::pad_function, C::rx_pad_config());
        C::template Pad<0>::release();
        /// Two engines of one transport must not name one DMA channel;
        /// what "the same channel" means is the family's DMA's business.
        { C::template engines_distinct<Pl011AbsentEngine, Pl011AbsentEngine>() }
            -> std::same_as<bool>;
        /// The line's two verbs, on the family's own controller. Nothing
        /// here raises the line by hand: every entry is a cause the
        /// block's own status shows.
        C::Interrupts::enable(line);
        C::Interrupts::disable(line);
    };

/**
 * WHICH CLOCK OF THIS FAMILY'S TREE IS UARTCLK - the half of the
 * contract that is a template over the family's clock types and so can
 * only be checked where a clock is in hand (Pl011Transport::init).
 */
template <typename C, typename Clock>
concept Pl011ChipClock = requires(Clock clock) {
    { C::uartclk_hz(clock) } -> std::same_as<uint32_t>;
};

// ---- frame vocabulary ---------------------------------------------------------

/// LCR_H.WLEN: the data bits of a frame (5..8).
enum class UartBits : uint8_t { five = 5, six = 6, seven = 7, eight = 8 };

enum class UartParity : uint8_t { none, even, odd };

struct UartFormat {
    UartBits bits = UartBits::eight;
    UartParity parity = UartParity::none;
    uint8_t stop_bits = 1;   ///< 1 or 2
};

constexpr bool uart_format_valid(const UartFormat& f) {
    const uint8_t bits = static_cast<uint8_t>(f.bits);
    return bits >= 5u && bits <= 8u && (f.stop_bits == 1u || f.stop_bits == 2u);
}

// ---- the baud arithmetic ----------------------------------------------------------

/// IBRD and FBRD: the divisor UARTCLK / (16 x baud) as an integer and
/// sixty-fourths.
struct UartDivisor {
    uint16_t integer;    ///< 1..65535
    uint8_t fraction;    ///< 0..63
};

/// The divisor for `baud` at UARTCLK `hz`, rounded to the nearest
/// sixty-fourth. Nothing when the rate is out of the generator's reach:
/// a baud above hz/16 (the integer part would be zero) or so low the
/// integer part overflows. The reach is the fractional divider's own
/// rule.
constexpr std::optional<UartDivisor> uart_divisor(uint32_t hz, uint32_t baud) {
    if (hz == 0u || baud == 0u) {
        return std::nullopt;
    }
    // (hz / (16 x baud)) x 64, rounded: hz x 4 / baud, + 0.5 in
    // sixty-fourths = + baud/2 before the division.
    const uint64_t sixty_fourths = (static_cast<uint64_t>(hz) * 4u + baud / 2u) / baud;
    const uint64_t integer = sixty_fourths / 64u;
    if (integer == 0u || integer > 65535u) {
        return std::nullopt;
    }
    return UartDivisor{.integer = static_cast<uint16_t>(integer),
                       .fraction = static_cast<uint8_t>(sixty_fourths % 64u)};
}

/// The rate a divisor really produces at `hz`.
constexpr uint32_t uart_actual_baud(uint32_t hz, UartDivisor d) {
    const uint64_t sixty_fourths = static_cast<uint64_t>(d.integer) * 64u + d.fraction;
    return sixty_fourths == 0u ? 0u
                               : static_cast<uint32_t>(static_cast<uint64_t>(hz) * 4u /
                                                       sixty_fourths);
}

/// The slowest UARTCLK that still produces `baud` (at least 16 x the
/// rate).
constexpr uint32_t uart_min_hz(uint32_t baud) { return baud * 16u; }

// ---- the register vocabulary --------------------------------------------------------

/// UARTFR, the flag register.
struct UartFlag {
    static constexpr uint32_t busy = 1u << 3;
    static constexpr uint32_t rx_empty = 1u << 4;
    static constexpr uint32_t tx_full = 1u << 5;
    static constexpr uint32_t rx_full = 1u << 6;
    static constexpr uint32_t tx_empty = 1u << 7;
};

/// The interrupt sources: one bit layout for IMSC, RIS, MIS and ICR.
struct UartInterrupt {
    static constexpr uint32_t modem_ri = 1u << 0;
    static constexpr uint32_t modem_cts = 1u << 1;
    static constexpr uint32_t modem_dcd = 1u << 2;
    static constexpr uint32_t modem_dsr = 1u << 3;
    static constexpr uint32_t rx = 1u << 4;
    static constexpr uint32_t tx = 1u << 5;
    static constexpr uint32_t rx_timeout = 1u << 6;
    static constexpr uint32_t frame = 1u << 7;
    static constexpr uint32_t parity = 1u << 8;
    static constexpr uint32_t brk = 1u << 9;
    static constexpr uint32_t overrun = 1u << 10;
    static constexpr uint32_t errors = frame | parity | brk | overrun;
    static constexpr uint32_t all = overrun | brk | parity | frame | rx_timeout | tx | rx |
                                    modem_dsr | modem_dcd | modem_cts | modem_ri;
};

/// UARTDR's error bits, stored with each received byte.
struct UartDataError {
    static constexpr uint32_t frame = 1u << 8;
    static constexpr uint32_t parity = 1u << 9;
    static constexpr uint32_t brk = 1u << 10;
    static constexpr uint32_t overrun = 1u << 11;
    static constexpr uint32_t dropped = frame | parity | brk;
};

/// UARTRSR, the same four errors as a STICKY status: any write clears
/// them, and the overrun is read here and never off an entry.
struct UartReceiveStatus {
    static constexpr uint32_t frame = 1u << 0;
    static constexpr uint32_t parity = 1u << 1;
    static constexpr uint32_t brk = 1u << 2;
    static constexpr uint32_t overrun = 1u << 3;
};

/// UARTCR: the enable and the two directions, and the loop-back.
struct UartControl {
    static constexpr uint32_t enable = 1u << 0;
    static constexpr uint32_t loopback = 1u << 7;
    static constexpr uint32_t tx_enable = 1u << 8;
    static constexpr uint32_t rx_enable = 1u << 9;
};

/// UARTLCR_H: the frame format, the FIFO enable and the break.
struct UartLineControl {
    static constexpr uint32_t brk = 1u << 0;
    static constexpr uint32_t parity_enable = 1u << 1;
    static constexpr uint32_t even_parity = 1u << 2;
    static constexpr uint32_t stop2 = 1u << 3;
    static constexpr uint32_t fifos = 1u << 4;
    static constexpr uint32_t word_length_lsb = 5;
};

/// UARTIFLS: where each FIFO's trigger level sits in the register.
struct UartTriggerField {
    static constexpr uint32_t tx_lsb = 0;
    static constexpr uint32_t tx_bits = 0x7u << tx_lsb;
    static constexpr uint32_t rx_lsb = 3;
    static constexpr uint32_t rx_bits = 0x7u << rx_lsb;
};

/// UARTIBRD and UARTFBRD: the two halves of the divisor, masked on the
/// way back out.
struct UartBaudField {
    static constexpr uint32_t integer = 0xFFFFu;
    static constexpr uint32_t fraction = 0x3Fu;
};

/// UARTDMACR: the two request enables.
struct UartDmaControl {
    static constexpr uint32_t rx = 1u << 0;
    static constexpr uint32_t tx = 1u << 1;
};

/// The FIFO trigger levels (UARTIFLS): the receive interrupt at or
/// above, the transmit interrupt at or below, this fraction of the
/// FIFO's depth.
enum class UartFifoLevel : uint8_t { eighth = 0, quarter = 1, half = 2, three_quarters = 3, seven_eighths = 4 };

// ---- the resource ----------------------------------------------------------------

template <Pl011Chip Chip, uint8_t n>
struct Pl011Uart {
    static_assert(n < Chip::instances, "this family has no PL011 with that number");
    Pl011Uart() = delete;

    using Regs = typename Chip::Regs;

    static constexpr uint8_t index = n;
    static constexpr uint8_t fifo_depth = Chip::fifo_depth;

    static Regs& regs() { return Chip::template regs<n>(); }
    static constexpr typename Chip::Irq irq() { return Chip::template irq<n>(); }

    /// The block from its reset state: held, released, ready.
    static bool reset() { return Chip::template reset<n>(); }
    static bool released() { return Chip::template released<n>(); }
    /// Into reset and gone.
    static void hold() { Chip::template hold<n>(); }

    static bool enabled() { return (regs().UARTCR & UartControl::enable) != 0u; }
    /// UARTEN with both directions (TXE, RXE); off clears the three and
    /// nothing else of UARTCR (the loop-back bit stays what it was).
    static void enable(bool on) {
        constexpr uint32_t bits = UartControl::enable | UartControl::tx_enable | UartControl::rx_enable;
        if (on) { Chip::set_bits(regs().UARTCR, bits); } else { Chip::clear_bits(regs().UARTCR, bits); }
    }

    /// The loop-back (UARTCR.LBE): the transmitter fed straight into the
    /// receiver, the RX pad ignored - a self-test with no wire. Refused
    /// (false, nothing written) while the UART is enabled, as every
    /// UARTCR change but the enable itself.
    static bool loopback(bool on) {
        if (enabled()) {
            return false;
        }
        if (on) { Chip::set_bits(regs().UARTCR, UartControl::loopback); }
        else { Chip::clear_bits(regs().UARTCR, UartControl::loopback); }
        return true;
    }
    static bool loopback() { return (regs().UARTCR & UartControl::loopback) != 0u; }

    /// The break (UARTLCR_H.BRK): TX held low after the current frame
    /// until cleared - a break longer than a frame is what a receiver
    /// reports as BE. Live: this one bit of LCR_H is meant to change
    /// under a running transmitter.
    static void break_send(bool on) {
        if (on) { Chip::set_bits(regs().UARTLCR_H, UartLineControl::brk); }
        else { Chip::clear_bits(regs().UARTLCR_H, UartLineControl::brk); }
    }
    static bool fifos_enabled() { return (regs().UARTLCR_H & UartLineControl::fifos) != 0u; }

    /// The line control: frame format and the FIFO enable, one LCR_H
    /// write - which also latches whatever IBRD/FBRD hold. Refused (false,
    /// nothing written) while the UART is enabled.
    static bool line_control(const UartFormat& f, bool fifos) {
        if (enabled() || !uart_format_valid(f)) {
            return false;
        }
        uint32_t v = (static_cast<uint32_t>(f.bits) - 5u) << UartLineControl::word_length_lsb;
        if (f.stop_bits == 2u) {
            v |= UartLineControl::stop2;
        }
        if (f.parity != UartParity::none) {
            v |= UartLineControl::parity_enable;
            if (f.parity == UartParity::even) {
                v |= UartLineControl::even_parity;
            }
        }
        if (fifos) {
            v |= UartLineControl::fifos;
        }
        regs().UARTLCR_H = v;
        return true;
    }

    /// IBRD/FBRD, then the dummy LCR_H write that makes them take.
    /// Refused while enabled.
    static bool divisor(UartDivisor d) {
        if (enabled()) {
            return false;
        }
        regs().UARTIBRD = d.integer;
        regs().UARTFBRD = d.fraction;
        regs().UARTLCR_H = regs().UARTLCR_H;
        return true;
    }
    static UartDivisor divisor() {
        return {.integer = static_cast<uint16_t>(regs().UARTIBRD & UartBaudField::integer),
                .fraction = static_cast<uint8_t>(regs().UARTFBRD & UartBaudField::fraction)};
    }

    /// The FIFO trigger levels.
    static void fifo_levels(UartFifoLevel rx, UartFifoLevel tx) {
        regs().UARTIFLS = (static_cast<uint32_t>(rx) << UartTriggerField::rx_lsb) |
                          (static_cast<uint32_t>(tx) << UartTriggerField::tx_lsb);
    }
    static UartFifoLevel rx_fifo_level() {
        return static_cast<UartFifoLevel>((regs().UARTIFLS & UartTriggerField::rx_bits) >>
                                          UartTriggerField::rx_lsb);
    }
    static UartFifoLevel tx_fifo_level() {
        return static_cast<UartFifoLevel>((regs().UARTIFLS & UartTriggerField::tx_bits) >>
                                          UartTriggerField::tx_lsb);
    }

    static uint32_t flags() { return regs().UARTFR; }
    static bool tx_full() { return (flags() & UartFlag::tx_full) != 0u; }
    static bool rx_empty() { return (flags() & UartFlag::rx_empty) != 0u; }
    static bool busy() { return (flags() & UartFlag::busy) != 0u; }

    /// One entry of the receive FIFO: the byte in bits 7..0, its error
    /// flags above (UartDataError).
    static uint32_t read_data() { return regs().UARTDR; }
    static void write_data(uint8_t b) { regs().UARTDR = b; }

    /// The sticky receive-status errors (UARTRSR); any write clears them.
    static uint32_t receive_status() { return regs().UARTRSR; }
    static void clear_receive_status() { regs().UARTRSR = 0u; }

    /// Interrupt mask (UARTIMSC), through the family's one-write set and
    /// clear: a handler and the loop can each touch their own bits.
    static void interrupts(uint32_t mask, bool on) {
        if (on) { Chip::set_bits(regs().UARTIMSC, mask); } else { Chip::clear_bits(regs().UARTIMSC, mask); }
    }
    static uint32_t interrupts() { return regs().UARTIMSC; }
    /// Masked status (UARTMIS): what is both raised and enabled.
    static uint32_t pending() { return regs().UARTMIS; }
    static uint32_t raw_pending() { return regs().UARTRIS; }
    static void clear_pending(uint32_t mask) { regs().UARTICR = mask; }

    /// The DMA request enables (UARTDMACR), for the engines' day.
    static void dma_requests(bool tx, bool rx) {
        regs().UARTDMACR = (tx ? UartDmaControl::tx : 0u) |
                           (rx ? UartDmaControl::rx : 0u);
    }
};

// ---- the task --------------------------------------------------------------------

/**
 * The interrupt-driven byte transport over Pl011Uart<Chip, n> (the file
 * header). The family's own header aliases it to `Uart<n, pins, ...>`
 * and carries the example in that family's spelling.
 *
 * TX policy: write_byte() has TRY semantics (false when the TX ring is
 * full, nothing counted - the caller decides whether to retry, drop or
 * block; print.hpp blocks). RX overflow (ring full, byte lost) IS
 * counted, as are the hardware error flags.
 *
 * Ring sizes: where the platform reads the ring's index atomically,
 * util/ring.hpp takes its lock-free path at every size and a larger ring
 * costs only RAM. The family's alias states the console-class defaults,
 * which are not a ceiling.
 */
template <Pl011Chip Chip, uint8_t n, typename Chip::Pins pins, uint32_t rx_size, uint32_t tx_size,
          typename TxEngine, typename RxEngine>
class Pl011Transport {
    using U = Pl011Uart<Chip, n>;

    static_assert(Chip::pins_valid(n, pins),
                  "these pads cannot carry this instance's signals: a UART's transmit "
                  "and receive pads are fixed per instance, each under the one function "
                  "that routes the UART to it - see the family's own pin table");
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type");
    static_assert(Chip::template engines_distinct<TxEngine, RxEngine>(),
                  "the transmit and receive engines must use DIFFERENT DMA channels");

    static constexpr uint8_t tx_pin = Chip::tx_pad(pins);
    static constexpr uint8_t rx_pin = Chip::rx_pad(pins);

    using TxPad = typename Chip::template Pad<tx_pin>;
    using RxPad = typename Chip::template Pad<rx_pin>;

    // One ring pair per instantiation (static inline -> .bss, no ctor).
    // The receive ring is told when a byte was lost (util/ring.hpp's
    // SkipRing), so its consumer's next look skips past the loss and
    // rx_skips() moves between two runs.
    static inline SkipRing<uint8_t, rx_size, typename Chip::Platform> m_rx{};
    static inline Ring<uint8_t, tx_size, typename Chip::Platform> m_tx{};

    // Error counters, written in the handler, read from the main loop.
    // A byte moves in one access on these cores; they wrap at 255. Written
    // as `x = x + 1` because compound ops on volatile are deprecated in
    // C++20.
    static inline volatile uint8_t m_rx_overruns = 0;   // RX ring full, byte lost
    static inline volatile uint8_t m_frame_errors = 0;  // FE: byte dropped
    static inline volatile uint8_t m_parity_errors = 0; // PE: byte dropped
    static inline volatile uint8_t m_break_errors = 0;  // BE: a break, dropped
    static inline volatile uint8_t m_hw_overruns = 0;   // OE: the FIFO was full when a frame landed (UARTRSR)
    static inline volatile uint16_t m_dma_faults = 0;   // blocks the engines threw away
    static inline uint32_t m_baud = 0;                  // for rebase()

public:
    /// Instances are empty tags for concept-based call sites (print(serial, ...)).
    constexpr Pl011Transport() = default;

    /// The resource underneath.
    using Resource = U;

    static constexpr bool has_tx_engine = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;

    // ---- lifecycle --------------------------------------------------------

    /// Bring the instance up: the block out of reset, the divisor and
    /// the line control, the FIFOs at their levels, the receive
    /// interrupts, the pads, the interrupt line.
    ///
    /// Call AFTER the main clock is set up and before interrupts are
    /// enabled globally; `clock` is the app's brio::Clock tag, and the
    /// family's traits say which of its rates is UARTCLK - so the
    /// divisor never comes from a second statement of the rate.
    ///
    /// False when the rate cannot be produced at this clock or the
    /// block did not come out of reset - the caller then knows the
    /// transport is NOT up, rather than printing into a ring nothing
    /// will drain.
    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, const UartFormat& format = {}) {
        static_assert(Pl011ChipClock<Chip, Clock>,
                      "the family's chip traits must say which rate of this Clock type is "
                      "UARTCLK (Pl011ChipClock's uartclk_hz)");
        static_assert(clock_follows<Clock, Pl011Transport>(),
                      "this Uart is initialized with a DynamicClock that does not "
                      "list it among its Users: it would keep the old baud after "
                      "a clock change");
        const std::optional<UartDivisor> d = uart_divisor(Chip::uartclk_hz(clock), baud);
        if (!d || !uart_format_valid(format)) {
            return false;
        }

        // init() STARTS the transport: nothing a previous life left in
        // the rings or the counters is this one's traffic. The handler
        // is the rings' other party, so its line goes down first -
        // Ring::clear() is the one verb that is not concurrent.
        Chip::Interrupts::disable(U::irq());
        m_rx.clear();
        m_tx.clear();
        clear_errors();
        m_baud = baud;

        if (!U::reset()) {
            return false;
        }
        // Everything below is written with the UART disabled, as the
        // block's rule wants; reset() left it so.
        U::fifo_levels(UartFifoLevel::half, UartFifoLevel::eighth);
        if (!U::divisor(*d) || !U::line_control(format, true)) {
            return false;
        }
        U::clear_pending(UartInterrupt::all);
        U::interrupts(UartInterrupt::all, false);
        if constexpr (!has_rx_engine) {
            U::interrupts(UartInterrupt::rx | UartInterrupt::rx_timeout, true);
        }
        // THE RX PAD FIRST, in the setup the family states for it,
        // before the receiver is enabled: where that setup is a pull-up
        // over a pad that comes up pulled down, a receiver enabled over
        // a low line takes a BREAK - a zero byte flagged BE that the
        // handler would drop by its flags but a receive ENGINE, which
        // moves bytes and not flags, would deliver at the head of every
        // run (measured, a wire from a silent peer on the pad). The TX
        // pad goes over only after the transmitter is enabled and its
        // line idles high: handing it over first would show whatever a
        // disabled block drives.
        RxPad::function(Chip::pad_function, Chip::rx_pad_config());
        U::enable(true);
        TxPad::function(Chip::pad_function, Chip::tx_pad_config());
        // And the receive FIFO is EMPTIED before an engine takes it:
        // whatever a peer put on the line between the reset and here
        // is not this life's traffic.
        while (!U::rx_empty()) {
            (void)U::read_data();
        }
        U::clear_receive_status();
        if constexpr (has_rx_engine) {
            // UNDER AN ENGINE THE ERRORS ARE THE VECTOR'S: the channel
            // moves an entry's byte and drops its flags (bits 8..11 of
            // UARTDR), and UARTRSR speaks for the last character READ -
            // the channel's - so the error interrupts, one a received
            // error, are where each one is counted (isr()).
            U::clear_pending(UartInterrupt::errors);
            U::interrupts(UartInterrupt::errors, true);
        }
        m_dma_faults = 0;
        if constexpr (has_tx_engine) {
            TxEngine::arm(&U::regs().UARTDR, Chip::template tx_request<n>());
        }
        if constexpr (has_rx_engine) {
            RxEngine::arm(&U::regs().UARTDR, Chip::template rx_request<n>());
            rearm_rx();
        }
        U::dma_requests(has_tx_engine, has_rx_engine);   // after the channels stand (port())

        Chip::Interrupts::enable(U::irq());
        return true;
    }

    /// The core clock changed (DynamicClock fan-out): keep the same bit
    /// rate at the new rate. Called BEFORE the clock changes, so the
    /// drain below runs at the rate the queued bytes were meant for. A
    /// byte being RECEIVED during the switch may still be garbled: the
    /// caller picks a quiet moment. Main context only.
    static void rebase(uint32_t hz) {
        const std::optional<UartDivisor> d = uart_divisor(hz, m_baud);
        if (!d) {
            return;   // the new rate cannot carry this baud: nothing better to do
        }
        drain();
        port(false);
        (void)U::divisor(*d);
        port(true);
    }

    /// Change the rate under the running port, once the TX side is
    /// idle. False, and nothing written, when the generator cannot
    /// express the rate. Main context only; the caller owns the
    /// agreement with the other end.
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const std::optional<UartDivisor> d = uart_divisor(hz, baud);
        if (!d) {
            return false;
        }
        drain();
        port(false);
        (void)U::divisor(*d);
        port(true);
        m_baud = baud;
        return true;
    }

    /// The slowest clock that can still produce `baud`.
    /// Change the frame format under the running port, once the TX side
    /// is idle; false and nothing written for a format the block has
    /// not got. Main context only.
    static bool set_format(const UartFormat& format) {
        if (!uart_format_valid(format)) {
            return false;
        }
        drain();
        port(false);
        const bool ok = U::line_control(format, true);
        port(true);
        return ok;
    }

    /// The loop-back on or off under the running port: the transmitter
    /// into the receiver, the RX pad ignored while on. Main context.
    static bool loopback(bool on) {
        drain();
        port(false);
        const bool ok = U::loopback(on);
        port(true);
        return ok;
    }

    static constexpr uint32_t min_hz_for(uint32_t baud) { return uart_min_hz(baud); }

    /// True if the generator can produce `baud` at `hz`.
    static constexpr bool can_baud(uint32_t hz, uint32_t baud) {
        return uart_divisor(hz, baud).has_value();
    }

    /// What the generator really produces at `hz` (the divisor read
    /// back), not what was asked for.
    static uint32_t actual_baud(uint32_t hz) { return uart_actual_baud(hz, U::divisor()); }

    /// Tear the transport down: the line masked, the block into reset,
    /// the pads back to their reset state. The rings keep what they
    /// hold until the next init().
    static void release() {
        Chip::Interrupts::disable(U::irq());
        U::interrupts(UartInterrupt::all, false);
        if constexpr (has_tx_engine) {
            TxEngine::stop();
        }
        if constexpr (has_rx_engine) {
            RxEngine::stop();
        }
        U::dma_requests(false, false);
        U::enable(false);
        TxPad::release();
        RxPad::release();
        U::hold();
    }

    // ---- the ISR body -----------------------------------------------------

    /// The instance's ONE interrupt body - call from the handler the
    /// family's crt names for this instance's line.
    ///
    /// Serves what UARTMIS reports, and nothing else: the receive FIFO
    /// on its level or its timeout, the transmit ring into the FIFO on
    /// the transmit edge (the file header's transmit-interrupt note).
    /// Nothing raises this line by hand, so every entry has a cause in
    /// UARTMIS; an entry for the receiver alone leaves the transmit side
    /// to its own edge.
    ///
    /// Returns true when the RX ring transitioned empty -> non-empty:
    /// the edge signal for kernel glue ("post RxActivity to the serial
    /// AO on true"). Every empty->non-empty transition reports true and
    /// the consumer only empties the ring by draining it, so no wakeup
    /// is ever lost. Plain (non-kernel) apps may ignore the return value.
    [[gnu::always_inline]] static bool isr() {
        const uint32_t active = U::pending();
        bool edge = false;
        if constexpr (has_rx_engine) {
            // The error interrupts, armed under an engine alone (init()):
            // each received error counted, its status cleared through
            // UARTICR - nothing here reads UARTDR, the channel's.
            const uint32_t errors = active & UartInterrupt::errors;
            if (errors != 0u) {
                if ((errors & UartInterrupt::frame) != 0u) { m_frame_errors = m_frame_errors + 1; }
                if ((errors & UartInterrupt::parity) != 0u) { m_parity_errors = m_parity_errors + 1; }
                if ((errors & UartInterrupt::brk) != 0u) { m_break_errors = m_break_errors + 1; }
                if ((errors & UartInterrupt::overrun) != 0u) {
                    m_hw_overruns = m_hw_overruns + 1;
                    // Frames the full FIFO refused: the ring told when
                    // the vector sees it, its consumer's next look skipping
                    // what the ring holds. The run in flight and the FIFO's
                    // content came before the loss and are published after
                    // that look, so a line among them can reach the
                    // consumer joined to the stream after the loss - a gap
                    // docs/pl011/README.md lists. A framed, parity-failed
                    // or break entry is moved by the channel and
                    // delivered: no loss.
                    m_rx.lost();
                }
                U::clear_pending(errors);
            }
        }
        if constexpr (!has_rx_engine) {
            if ((active & (UartInterrupt::rx | UartInterrupt::rx_timeout)) != 0u) {
                edge = receive((active & UartInterrupt::rx) != 0u);
            }
        }
        if constexpr (!has_tx_engine) {
            if ((active & UartInterrupt::tx) != 0u) {
                feed();
            }
        }
        (void)active;
        return edge;
    }

    /**
     * The ISR body of the DMA LINE this transport's engines report on -
     * the app binds that line's handler to it.
     *
     * Each engine reads only its own channel's status, so a line other
     * channels of the program report on is shared safely. A transmit
     * completion releases exactly the block's bytes from the ring and
     * starts the next run; a receive completion publishes and re-arms
     * here - unless harvest() met it first, under its guard, and served
     * it there: a completion is acted on ONCE.
     *
     * Returns the receive ring's empty -> non-empty edge across the
     * completion's publish, the answer isr() gives: the same kernel glue
     * posts RxActivity on it, so a stream that fills runs is told once a
     * run with no poll. A BURST SHORTER THAN THE RUN HAS NO EDGE HERE,
     * and none anywhere on this block: the receive time-out needs a
     * character waiting in the FIFO, and the channel's single request
     * leaves none (measured: UARTRIS.RTRIS never rises under an engine,
     * docs/pl011/README.md) - a partial run is harvest()'s to publish,
     * and the interrupt receiver, one entry a FIFO level plus the
     * time-out's, is the burst path with an edge.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        bool edge = false;
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service();
            if ((f & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                m_dma_faults = m_dma_faults + 1u;
            } else if ((f & TxEngine::flag_complete) != 0u) {
                m_tx.consume(static_cast<typename decltype(m_tx)::index_t>(TxEngine::complete()));
                pump_tx();
            }
        }
        if constexpr (has_rx_engine) {
            const uint8_t f = RxEngine::service();
            if ((f & RxEngine::flag_error) != 0u) {
                (void)RxEngine::abandon();
                m_dma_faults = m_dma_faults + 1u;
            } else if ((f & RxEngine::flag_complete) != 0u) {
                // The run filled: published and re-armed HERE, not left to
                // harvest() - at 3 Mbaud a 32-deep FIFO overflows 100 us
                // after the run ends, and the credits the overflow leaves
                // behind would be spent reading nothing (measured).
                const bool was_empty = m_rx.empty();
                publish_rx();
                rearm_rx();
                edge = was_empty && !m_rx.empty();
            }
        }
        return edge;
    }

    /**
     * Ask the receive engine what has arrived, and publish it (the file
     * header). Returns the same edge isr() does - the ring went from
     * empty to non-empty. False, and free, without an engine.
     */
    static bool harvest() {
        if constexpr (!has_rx_engine) {
            return false;
        } else {
            // The ring's producer side is shared with the line's handler
            // (a completion publishes and re-arms there): under the guard.
            typename Chip::Guard guard;
            const bool was_empty = m_rx.empty();
            // WHETHER THE RUN HAS ENDED IS ASKED BEFORE ITS COUNT IS READ,
            // and the answer is used as it stood. Asked after, a run whose
            // last byte lands between the two reads is seen ended with
            // that byte uncounted: it is re-armed over, the new run's
            // first byte lands on the old run's last, and ONE BYTE OF THE
            // STREAM IS GONE (measured: one of 4096 at 3 Mbaud, at the
            // end of a short run by the ring's wrap, one run in five when
            // this code runs cold out of the flash). Asked before, an
            // ended run has its final count, and a run that ends after
            // the question is simply left to the line's handler.
            const bool ended = RxEngine::idle();
            if (ended) {
                // A completion the masked line still owes is served HERE:
                // this verb is about to publish and re-arm that run, and a
                // completion is acted on ONCE - left standing, its flag
                // would send the handler to re-arm over the run just begun.
                const uint8_t owed = RxEngine::service();
                if ((owed & RxEngine::flag_error) != 0u) {
                    (void)RxEngine::abandon();
                    m_dma_faults = m_dma_faults + 1u;
                }
            }
            publish_rx();
            if (ended || RxEngine::capacity() == 0u) {
                rearm_rx();
            }
            return was_empty && !m_rx.empty();
        }
    }

    /// Blocks the engines threw away after a bus error. Zero, and free,
    /// without an engine.
    static uint16_t dma_faults() { return m_dma_faults; }

    // ---- byte transport (satisfies ByteSink / ByteSource) -----------------

    /// Try to send one byte; false when the TX ring is full. Without a
    /// transmit engine (the file header's three rules): straight into
    /// UARTDR when nothing is queued and the FIFO has room, else into the
    /// ring with TXIM armed, and a refusal writes nothing. With one, the
    /// byte is queued and the engine pumped - on a refusal too, because a
    /// block abandoned after a bus error starts no next one, and a
    /// refused write is the one call a blocked print keeps making.
    static bool write_byte(uint8_t b) {
        if constexpr (has_tx_engine) {
            const bool queued = m_tx.push(b);
            pump_tx();
            return queued;
        } else {
            if (m_tx.empty() && !U::tx_full()) {
                U::write_data(b);
                return true;
            }
            if (!m_tx.push(b)) {
                return false;
            }
            // AFTER the push: a handler that ran between the two and
            // emptied the ring leaves at most a mask armed over an empty
            // ring, one entry that disarms it. Armed BEFORE, that handler
            // could disarm it over the byte just queued.
            U::interrupts(UartInterrupt::tx, true);
            return true;
        }
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

    /// Send a run of bytes in BULK: without a transmit engine, as much of
    /// it as the FIFO has room for goes straight into UARTDR when nothing
    /// is queued (write_byte()'s first rule, a run at a time); the rest
    /// is copied into the ring's own free run, and TXIM armed ONCE if any
    /// of it was queued. With an engine, all of it is queued and the
    /// engine pumped once. Returns how many were taken (short of
    /// `src.size()` when the ring filled).
    static uint32_t write_bulk(std::span<const uint8_t> src) {
        uint32_t done = 0;
        if constexpr (!has_tx_engine) {
            if (m_tx.empty()) {
                while (done < src.size() && !U::tx_full()) {
                    U::write_data(src[done]);
                    ++done;
                }
            }
        }
        [[maybe_unused]] const uint32_t direct = done;
        while (done < src.size()) {
            const auto room = m_tx.write_span();
            if (room.empty()) {
                break;
            }
            const uint32_t want = static_cast<uint32_t>(src.size()) - done;
            const uint32_t take =
                want < room.size() ? want : static_cast<uint32_t>(room.size());
            copy_run(room.data(), src.data() + done, take);
            m_tx.publish(static_cast<typename decltype(m_tx)::index_t>(take));
            done += take;
        }
        if constexpr (has_tx_engine) {
            pump_tx();
        } else if (done != direct) {
            U::interrupts(UartInterrupt::tx, true);   // after the publish: write_byte()'s order
        }
        return done;
    }

    /// Take a run of received bytes in BULK, the mirror of write_bulk().
    static uint32_t read_bulk(std::span<uint8_t> dst) {
        uint32_t done = 0;
        while (done < dst.size()) {
            const auto run = m_rx.read_span();
            if (run.empty()) {
                break;
            }
            const uint32_t want = static_cast<uint32_t>(dst.size()) - done;
            const uint32_t take =
                want < run.size() ? want : static_cast<uint32_t>(run.size());
            for (uint32_t i = 0; i < take; ++i) {
                dst[done + i] = run[i];
            }
            m_rx.consume(static_cast<typename decltype(m_rx)::index_t>(take));
            done += take;
        }
        return done;
    }

    /// The received bytes IN PLACE: the contiguous run ready to be read,
    /// never wrapping - the receive ring's consumer half under the ring's
    /// own names. With consume() it is util/stream.hpp's SpanSource,
    /// which SerialPort drains a run at a time with no copy at all.
    static std::span<const uint8_t> read_span() { return m_rx.read_span(); }

    /// Release the first `count` bytes of read_span(), oldest first, clamped
    /// to what is queued.
    static void consume(uint32_t count) {
        constexpr uint32_t most = decltype(m_rx)::capacity();
        m_rx.consume(static_cast<typename decltype(m_rx)::index_t>(count < most ? count : most));
    }

    static bool rx_pending() { return !m_rx.empty(); }
    /// Nothing queued, no block in flight, nothing in the FIFO, nothing
    /// in the shifter.
    static bool tx_idle() {
        if constexpr (has_tx_engine) {
            return m_tx.empty() && !TxEngine::busy() && !U::busy();
        } else {
            return m_tx.empty() && !U::busy();
        }
    }

    // ---- diagnostics ------------------------------------------------------

    static uint8_t rx_overruns() { return m_rx_overruns; }
    static uint8_t frame_errors() { return m_frame_errors; }
    static uint8_t parity_errors() { return m_parity_errors; }
    static uint8_t break_errors() { return m_break_errors; }
    static uint8_t hw_overruns() { return m_hw_overruns; }

    /// Every byte the line carried that the receive ring will not deliver,
    /// since the program started, NEVER CLEARED (modulo 2^32) -
    /// util/stream.hpp's SkippingSource, the epoch util/serial_port.hpp
    /// compares at every run: the receive ring's skips (SkipRing), one at
    /// the consumer's look after an entry dropped for its flags or refused
    /// by a full ring, or an overrun - the ring discarding what it held.
    static uint32_t rx_skips() { return m_rx.skips(); }
    static void clear_errors() {
        m_rx_overruns = 0;
        m_frame_errors = 0;
        m_parity_errors = 0;
        m_break_errors = 0;
        m_hw_overruns = 0;
    }

private:
    /// The port off or on WITH ITS DMA REQUESTS: the requests cleared
    /// before the UART is disabled and set again after it is enabled,
    /// because the PL011 re-asserts a request when the UART comes back
    /// with RXDMAE set - and a request on an EMPTY receive FIFO is one
    /// credit the DMA spends reading nothing (measured: a zero byte at
    /// the head of every run after a loop-back or a format change).
    static void port(bool on) {
        if (on) {
            U::enable(true);
            U::dma_requests(has_tx_engine, has_rx_engine);
        } else {
            U::dma_requests(false, false);
            U::enable(false);
        }
    }

    /// Start the next contiguous run of the TX ring on the engine, if it
    /// is idle and there is one. THE GUARD COVERS THE CLAIM AND NOTHING
    /// ELSE: the thread and the completion on the DMA line both start
    /// blocks, and "is the channel mine" is the one decision they race
    /// for. The run is read and the block programmed with the mask down,
    /// because a claimed channel has no block in flight - no completion
    /// can consume the ring or start another block under the loader - and
    /// the run is read AFTER the claim, so it never holds bytes a block
    /// that ended meanwhile has already sent.
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            {
                typename Chip::Guard guard;
                if (!TxEngine::claim()) {
                    return;
                }
            }
            const auto run = m_tx.read_span();
            if (run.empty()) {
                TxEngine::unclaim();
                return;
            }
            (void)TxEngine::launch(run);
        }
    }

    /// write_bulk()'s copy into the ring's free run: THE RUNTIME'S memcpy
    /// WHERE ITS FIXED COST PAYS - a run of the family's `copy_crossover`
    /// bytes or more whose two ends share their word alignment, which is
    /// when its word block runs at all - and the byte loop below it: two
    /// pointers and no index, the test at the bottom, a load, a store,
    /// two steps and one branch a byte. `count` is at least one.
    [[gnu::always_inline]] static void copy_run(uint8_t* to, const uint8_t* from, uint32_t count) {
        if (count >= Chip::copy_crossover &&
            ((reinterpret_cast<uintptr_t>(to) ^ reinterpret_cast<uintptr_t>(from)) & 3u) == 0u) {
            std::memcpy(to, from, count);
            return;
        }
        uint8_t* const end = to + count;
        do {
            *to++ = *from++;
        } while (to != end);
    }

    /// What the receive engine has landed since the last look, handed to
    /// the ring's consumer.
    static void publish_rx() {
        if constexpr (has_rx_engine) {
            const uint32_t fresh = RxEngine::take();
            if (fresh != 0u) {
                m_rx.publish(static_cast<typename decltype(m_rx)::index_t>(fresh));
            }
        }
    }

    /// Point the receive engine at the ring's next free run. No room is
    /// bytes lost before they arrive, counted as the ring overrun it is.
    static void rearm_rx() {
        if constexpr (has_rx_engine) {
            const auto room = m_rx.write_span();
            if (room.empty()) {
                m_rx_overruns = m_rx_overruns + 1;
                return;
            }
            (void)RxEngine::start(room.data(), static_cast<uint32_t>(room.size()));
        }
    }

    /// The receive level init() programs, half the FIFO, in entries.
    static constexpr uint32_t rx_level_entries = Chip::fifo_depth / 2u;

    /// One receive FIFO entry into the ring, its own error flags
    /// attributed to it: a framed, parity-failed or break entry counted,
    /// dropped and reported lost; an entry the full ring refuses counted
    /// and reported the same - each on its rare path.
    [[gnu::always_inline]] static void take(uint32_t entry) {
        if ((entry & UartDataError::dropped) != 0u) [[unlikely]] {
            if ((entry & UartDataError::frame) != 0u) {
                m_frame_errors = m_frame_errors + 1;
            }
            if ((entry & UartDataError::parity) != 0u) {
                m_parity_errors = m_parity_errors + 1;
            }
            if ((entry & UartDataError::brk) != 0u) {
                m_break_errors = m_break_errors + 1;
            }
            m_rx.lost();
            return;
        }
        if (!m_rx.push(static_cast<uint8_t>(entry))) [[unlikely]] {
            m_rx_overruns = m_rx_overruns + 1;
            m_rx.lost();
        }
    }

    /// Drain the receive FIFO into the ring. Returns the ring's empty ->
    /// non-empty edge.
    ///
    /// AN ENTRY FOR THE LEVEL READS THE LEVEL BLIND. The receive interrupt
    /// stands while the FIFO holds its trigger level or more (RP2040
    /// 4.2.6.2), and this handler is the FIFO's one reader, so an entry
    /// with RXMIS set finds `rx_level_entries` characters at least and
    /// takes them with no UARTFR read between - one peripheral load a
    /// character where the flag test made two, the loads the per-byte
    /// cost is made of on this block. The rest, and an entry for the
    /// time-out alone, go with the flag tested before every read.
    ///
    /// A LOSS IS REPORTED WHERE IT IS MET. An entry dropped or refused is
    /// reported to the ring on its rare path, and an overrun after the
    /// drain: where in the stream it fell is nobody's business, since the
    /// consumer's look after it skips everything queued (util/ring.hpp's
    /// SkipRing), and nothing is kept across the drain for it.
    [[gnu::always_inline]] static bool receive(bool level) {
        const bool was_empty = m_rx.empty();
        if (level) {
            for (uint32_t i = 0; i < rx_level_entries; ++i) {
                take(U::read_data());
            }
        }
        while (!U::rx_empty()) {
            take(U::read_data());
        }
        // THE OVERRUN IS READ FROM UARTRSR, NOT FROM THE ENTRIES: the OE
        // bit of a FIFO entry is a LIVE condition (cleared once there is
        // an empty space in the FIFO) - the first read makes the space,
        // so the entries never carry it (measured); the status
        // register's OE is sticky until written, and one overrun event
        // is one count, however many frames it swallowed.
        if ((U::receive_status() & UartReceiveStatus::overrun) != 0u) [[unlikely]] {
            m_hw_overruns = m_hw_overruns + 1;
            U::clear_receive_status();
            m_rx.lost();
        }
        U::clear_pending(UartInterrupt::rx_timeout);
        return was_empty && !m_rx.empty();
    }

    /// Refill the FIFO from the ring on the transmit edge, a contiguous
    /// run at a time: the run copied while the FIFO takes it and released
    /// with one consume. The ring's consumer is this function alone.
    ///
    /// When the FIFO fills with bytes still queued, TXIM stays armed (the
    /// push that queued them armed it) and the next fall through the
    /// level brings the handler back. When the ring runs dry TXIM is
    /// disarmed and TXRIS IS LEFT AS IT STANDS: cleared here, with the
    /// FIFO at or below its level, nothing would fire the arming a
    /// write_byte() caught between its empty() and its push() makes next
    /// - a byte queued behind a mask that never fires (the file header).
    /// A TXRIS latched under a disarmed mask costs nothing: the next write
    /// that takes the FIFO above its level clears it.
    [[gnu::always_inline]] static void feed() {
        using index_t = typename decltype(m_tx)::index_t;
        for (;;) {
            const auto run = m_tx.read_span();
            if (run.empty()) {
                U::interrupts(UartInterrupt::tx, false);
                return;
            }
            uint32_t moved = 0;
            while (moved < run.size() && !U::tx_full()) {
                U::write_data(run[moved]);
                ++moved;
            }
            m_tx.consume(static_cast<index_t>(moved));
            if (moved < run.size()) {
                return;
            }
        }
    }

    /// Wait, bounded, for the ring and the shifter to empty: what a
    /// divisor change needs first. A full 256-byte ring at 9600 baud is
    /// a quarter of a second; the budgets are generous at any rate.
    static void drain() {
        constexpr uint32_t ring_drain_spins = 8'000'000u;
        constexpr uint32_t frame_spins = 200'000u;
        uint32_t spins = ring_drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = frame_spins;
        while (U::busy() && spins-- != 0u) {
        }
    }
};

} // namespace brio
