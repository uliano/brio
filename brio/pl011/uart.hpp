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
 *  - THE TRANSMIT INTERRUPT IS A TRANSITION, NOT A LEVEL: it asserts
 *    when the FIFO falls THROUGH its trigger level, and enabling it over
 *    an empty FIFO raises nothing. So no verb of this task waits for
 *    TXIM to start a transmission: write_byte() queues the byte and
 *    PENDS THE LINE in the family's interrupt controller, the handler
 *    moves the ring into the FIFO until the FIFO is full or the ring is
 *    empty, and only when bytes remain queued is TXIM armed - the FIFO
 *    draining through the level is then a real transition. The ring's
 *    consumer is the handler alone;
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
 * pump_tx() starts a block over the ring's contiguous run whenever the
 * engine is idle; the block's completion - on the DMA line the engine
 * reports on, dma_isr() - releases exactly that run and starts the next.
 * The transmit FIFO's own data request (UARTDMACR.TXDMAE) paces it,
 * credit by credit, so no kick is ever needed. With a RECEIVE engine the
 * receive interrupts stay off and the engine fills the ring's free run
 * straight from UARTDR; what has arrived is read off the engine's own
 * count by harvest(), a VERB the owner calls at its own pace (a kernel
 * TimeEvent every few ticks is the shape), which publishes it and
 * re-arms the run - and reads UARTRSR once per harvest, since the engine
 * moves BYTES and the per-entry error flags of UARTDR are not among
 * them: an error is counted against the harvested run, not a byte. The
 * DMA and the handler never touch one FIFO at once.
 *
 * NOT built, each with its reason in docs/pl011/README.md: hardware flow
 * control, IrDA, the modem status inputs and outputs, the stick-parity
 * bit. The slots for DMA engines ARE here, empty, so the type's shape
 * does not move when a family fills them.
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <optional>
#include <span>

#include "kernel/platform.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"

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
        /// That controller: enable / disable / raise one line. A family
        /// may carry an NVIC under one architecture and something else
        /// under another, so this file never names one.
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
        /// The line's three verbs, on the family's own controller.
        C::Interrupts::enable(line);
        C::Interrupts::disable(line);
        C::Interrupts::set_pending(line);
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
    static inline Ring<uint8_t, rx_size, typename Chip::Platform> m_rx{};
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
    /// Serves what UARTMIS reports and, whatever it reports, moves the
    /// transmit ring into the FIFO: the entry may be the pend a
    /// write_byte() raised, which shows nothing in the UART's own
    /// status (the file header's transmit-interrupt note).
    ///
    /// Returns true when the RX ring transitioned empty -> non-empty:
    /// the edge signal for kernel glue ("post RxActivity to the serial
    /// AO on true"). Every empty->non-empty transition reports true and
    /// the consumer only empties the ring by draining it, so no wakeup
    /// is ever lost. Plain (non-kernel) apps may ignore the return value.
    [[gnu::always_inline]] static bool isr() {
        const uint32_t active = U::pending();
        bool edge = false;
        if constexpr (!has_rx_engine) {
            if ((active & (UartInterrupt::rx | UartInterrupt::rx_timeout)) != 0u) {
                edge = receive();
            }
        } else {
            (void)active;
        }
        if constexpr (!has_tx_engine) {
            feed();
        }
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
     * Returns true when something of this transport's was served.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        bool mine = false;
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service();
            if ((f & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                m_dma_faults = m_dma_faults + 1u;
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
                m_dma_faults = m_dma_faults + 1u;
                mine = true;
            } else if ((f & RxEngine::flag_complete) != 0u) {
                // The run filled: published and re-armed HERE, not left to
                // harvest() - at 3 Mbaud a 32-deep FIFO overflows 100 us
                // after the run ends, and the credits the overflow leaves
                // behind would be spent reading nothing (measured).
                publish_rx();
                rearm_rx();
                mine = true;
            }
        }
        return mine;
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
            const uint32_t status = U::receive_status();
            if (status != 0u) {
                if ((status & UartReceiveStatus::frame) != 0u) { m_frame_errors = m_frame_errors + 1; }
                if ((status & UartReceiveStatus::parity) != 0u) { m_parity_errors = m_parity_errors + 1; }
                if ((status & UartReceiveStatus::brk) != 0u) { m_break_errors = m_break_errors + 1; }
                if ((status & UartReceiveStatus::overrun) != 0u) { m_hw_overruns = m_hw_overruns + 1; }
                U::clear_receive_status();
            }
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

    /// Try to queue one byte for transmission; false when the TX ring is
    /// full. The line is PENDED, never written to from here: the handler
    /// is the FIFO's one feeder.
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            nudge();   // a refused byte still nudges: see below
            return false;
        }
        nudge();
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

    /// Queue a run of bytes in BULK: copy straight into the ring's own
    /// free run and nudge the handler ONCE. Returns how many were queued
    /// (short of `src.size()` when the ring filled).
    static uint32_t write_bulk(std::span<const uint8_t> src) {
        uint32_t done = 0;
        while (done < src.size()) {
            const auto room = m_tx.write_span();
            if (room.empty()) {
                break;
            }
            const uint32_t want = static_cast<uint32_t>(src.size()) - done;
            const uint32_t take =
                want < room.size() ? want : static_cast<uint32_t>(room.size());
            for (uint32_t i = 0; i < take; ++i) {
                room[i] = src[done + i];
            }
            m_tx.publish(static_cast<typename decltype(m_tx)::index_t>(take));
            done += take;
        }
        nudge();
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

    /// What a queued byte does to get moving: pend the handler (the
    /// FIFO's one feeder) without an engine, start a block with one.
    static void nudge() {
        if constexpr (has_tx_engine) {
            pump_tx();
        } else {
            Chip::Interrupts::set_pending(U::irq());
        }
    }

    /// Start the next contiguous run of the TX ring on the engine, if it
    /// is idle and there is one. Under the guard: the completion path
    /// runs on the DMA line.
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            typename Chip::Guard guard;
            if (TxEngine::busy()) {
                return;
            }
            const auto run = m_tx.read_span();
            if (run.empty()) {
                return;
            }
            (void)TxEngine::start(run.data(), static_cast<uint32_t>(run.size()));
        }
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

    /// Drain the receive FIFO into the ring, attributing each entry's
    /// error flags to its own byte. Returns the ring's empty -> non-empty
    /// edge.
    [[gnu::always_inline]] static bool receive() {
        const bool was_empty = m_rx.empty();
        while (!U::rx_empty()) {
            const uint32_t entry = U::read_data();
            if ((entry & UartDataError::dropped) != 0u) {
                if ((entry & UartDataError::frame) != 0u) {
                    m_frame_errors = m_frame_errors + 1;
                }
                if ((entry & UartDataError::parity) != 0u) {
                    m_parity_errors = m_parity_errors + 1;
                }
                if ((entry & UartDataError::brk) != 0u) {
                    m_break_errors = m_break_errors + 1;
                }
                continue;
            }
            if (!m_rx.push(static_cast<uint8_t>(entry))) {
                m_rx_overruns = m_rx_overruns + 1;
            }
        }
        // THE OVERRUN IS READ FROM UARTRSR, NOT FROM THE ENTRIES: the OE
        // bit of a FIFO entry is a LIVE condition (cleared once there is
        // an empty space in the FIFO) - the first read makes the space,
        // so the entries never carry it (measured); the status
        // register's OE is sticky until written, and one overrun event
        // is one count, however many frames it swallowed.
        if ((U::receive_status() & UartReceiveStatus::overrun) != 0u) {
            m_hw_overruns = m_hw_overruns + 1;
            U::clear_receive_status();
        }
        U::clear_pending(UartInterrupt::rx_timeout);
        return was_empty && !m_rx.empty();
    }

    /// Move the transmit ring into the FIFO while both allow, and leave
    /// TXIM armed exactly when bytes remain queued: the FIFO draining
    /// through its level is then a real transition that brings the
    /// handler back. The ring's consumer is this function alone.
    [[gnu::always_inline]] static void feed() {
        while (!U::tx_full()) {
            const auto v = m_tx.pop();
            if (!v) {
                U::interrupts(UartInterrupt::tx, false);
                U::clear_pending(UartInterrupt::tx);
                return;
            }
            U::write_data(*v);
        }
        U::interrupts(UartInterrupt::tx, true);
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
