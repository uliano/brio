/*
 * uart.hpp
 *
 * The UART (datasheet 4.2: two ARM PL011s) in the two strata every brio
 * serial driver has (docs/design/serial.md):
 *
 *  Pl011<n>              the RESOURCE, named for the IP the chapter names
 *                        (table 280 calls the function "the internal
 *                        PL011 UART peripherals"): which instance, where
 *                        its registers are, its reset bit, its NVIC line,
 *                        the line control, the two baud divisors and the
 *                        write that latches them, the FIFOs and their
 *                        trigger levels, the interrupt mask/status/clear
 *                        trio, the flag register;
 *  Uart<n, pins, ...>    the TASK: the asynchronous byte transport with
 *                        ring buffers and one ISR body - every console's
 *                        personality, a ByteTransport for print() and
 *                        SerialPort. Its public surface is every target's
 *                        Uart surface, which is what lets
 *                        util/serial_port.hpp and print() compile here
 *                        untouched.
 *
 * WHAT IS THIS CHIP'S about the PL011, and shapes the task:
 *  - UARTCLK is clk_peri (2.15.3.1), which rp2040/clock.hpp feeds from
 *    clk_sys undivided: the divisor is computed from Clock::pclk_hz and
 *    never from a second statement of the rate. The baud rate divisor is
 *    UARTCLK / (16 x baud) as a 16-bit integer plus a 6-bit fraction
 *    (4.2.7.1); the two registers take effect only on the NEXT write of
 *    UARTLCR_H, so the divisor is written before the line control, and
 *    a divisor change alone is followed by a dummy LCR_H write;
 *  - LCR_H, IBRD and FBRD are written with the UART DISABLED (4.2.7,
 *    the PL011's rule): the task disables around every configuration;
 *  - BOTH FIFOs ARE 32 DEEP (4.2.2.4/5), and the receive FIFO stores
 *    the framing, parity and break flags WITH EACH BYTE (bits 8..10 of
 *    UARTDR), so a corrupted byte is dropped precisely; the overrun bit
 *    (bit 11) says a byte was lost AFTER the one read, and that one is
 *    good;
 *  - THE TRANSMIT INTERRUPT IS A TRANSITION, NOT A LEVEL (4.2.6.3): it
 *    asserts when the FIFO falls THROUGH its trigger level, and enabling
 *    it over an empty FIFO raises nothing. So no verb of this task
 *    waits for TXIM to start a transmission: write_byte() queues the
 *    byte and PENDS THE LINE in the NVIC, the handler moves the ring
 *    into the FIFO until the FIFO is full or the ring is empty, and
 *    only when bytes remain queued is TXIM armed - the FIFO draining
 *    through the level is then a real transition. The ring's consumer
 *    is the handler alone;
 *  - the receive side arms RXIM (the FIFO at its trigger level) AND
 *    RTIM (4.2.6.4: bytes waiting and the line idle for 32 bit periods),
 *    so a short burst is delivered after one character time and a long
 *    one every sixteen bytes; the handler drains the FIFO whole;
 *  - the line's interrupt is ONE NVIC line per instance (UART0_IRQ,
 *    UART1_IRQ, table 2.3.2), the app binds isr_uart0 / isr_uart1.
 *
 * THE PINS. UART0 and UART1 each reach four TX pads and four RX pads
 * under function 2 (2.19.2, table 279), fixed per instance: UART0 TX on
 * GPIO 0, 12, 16, 28 and RX on 1, 13, 17, 29; UART1 TX on 4, 8, 20, 24
 * and RX on 5, 9, 21, 25. `uart_pins_valid` is that table, and a pin
 * set that is not in it does not compile. CTS/RTS pads exist (the next
 * pins of each group) and hardware flow control is a gap, below.
 *
 * NOT built (docs/rp2040/uart.md's list): hardware flow control, IrDA,
 * the modem status inputs, the loop-back (CR.LBE) and the DMA slots'
 * engines - the slots are here, empty, so the type's shape does not
 * move when they arrive.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <span>

#include "rp2040/device.hpp"

#include "rp2040/clock.hpp"
#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/resets.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"

namespace brio {

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

/// The two pads of an asynchronous link, each named with its function.
struct UartPins {
    PinSel tx;
    PinSel rx;
};

/// Table 279's UART columns: is `pin` instance n's TX (or RX) pad?
constexpr bool uart_tx_pin(uint8_t n, uint8_t pin) {
    if (pin >= gpio_count || (pin % 4u) != 0u) {
        return false;
    }
    // Groups of four pins alternate UART0, UART1, UART1, UART0, ...
    const uint8_t group = pin / 4u;
    const uint8_t instance = static_cast<uint8_t>(((group + 1u) / 2u) % 2u);
    return instance == n;
}
constexpr bool uart_rx_pin(uint8_t n, uint8_t pin) {
    return (pin % 4u) == 1u && uart_tx_pin(n, static_cast<uint8_t>(pin - 1u));
}

/// A legal pin set for instance n: the pads under function 2 the table
/// gives that instance, one of each direction.
constexpr bool uart_pins_valid(uint8_t n, const UartPins& p) {
    return p.tx.function == PinFunction::uart && p.rx.function == PinFunction::uart &&
           uart_tx_pin(n, p.tx.pin) && uart_rx_pin(n, p.rx.pin);
}

// ---- the baud arithmetic (4.2.7.1) ------------------------------------------------

/// IBRD and FBRD: the divisor UARTCLK / (16 x baud) as an integer and
/// sixty-fourths.
struct UartDivisor {
    uint16_t integer;    ///< 1..65535
    uint8_t fraction;    ///< 0..63
};

/// The divisor for `baud` at UARTCLK `hz`, rounded to the nearest
/// sixty-fourth. Nothing when the rate is out of the generator's reach:
/// a baud above hz/16 (the integer part would be zero) or so low the
/// integer part overflows. The reach is 4.2.3.1's own rule.
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

/// The slowest UARTCLK that still produces `baud` (4.2.3.1: at least
/// 16 x the rate).
constexpr uint32_t uart_min_hz(uint32_t baud) { return baud * 16u; }

// ---- the register vocabulary --------------------------------------------------------

/// UARTFR, the flag register.
struct UartFlag {
    static constexpr uint32_t busy = UART_UARTFR_BUSY_BITS;
    static constexpr uint32_t rx_empty = UART_UARTFR_RXFE_BITS;
    static constexpr uint32_t tx_full = UART_UARTFR_TXFF_BITS;
    static constexpr uint32_t rx_full = UART_UARTFR_RXFF_BITS;
    static constexpr uint32_t tx_empty = UART_UARTFR_TXFE_BITS;
};

/// The interrupt sources: one bit layout for IMSC, RIS, MIS and ICR.
struct UartInterrupt {
    static constexpr uint32_t rx = UART_UARTIMSC_RXIM_BITS;
    static constexpr uint32_t tx = UART_UARTIMSC_TXIM_BITS;
    static constexpr uint32_t rx_timeout = UART_UARTIMSC_RTIM_BITS;
    static constexpr uint32_t frame = UART_UARTIMSC_FEIM_BITS;
    static constexpr uint32_t parity = UART_UARTIMSC_PEIM_BITS;
    static constexpr uint32_t brk = UART_UARTIMSC_BEIM_BITS;
    static constexpr uint32_t overrun = UART_UARTIMSC_OEIM_BITS;
    static constexpr uint32_t errors = frame | parity | brk | overrun;
    static constexpr uint32_t all = UART_UARTIMSC_OEIM_BITS | UART_UARTIMSC_BEIM_BITS |
                                    UART_UARTIMSC_PEIM_BITS | UART_UARTIMSC_FEIM_BITS |
                                    UART_UARTIMSC_RTIM_BITS | UART_UARTIMSC_TXIM_BITS |
                                    UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_DSRMIM_BITS |
                                    UART_UARTIMSC_DCDMIM_BITS | UART_UARTIMSC_CTSMIM_BITS |
                                    UART_UARTIMSC_RIMIM_BITS;
};

/// UARTDR's error bits, stored with each received byte.
struct UartDataError {
    static constexpr uint32_t frame = UART_UARTDR_FE_BITS;
    static constexpr uint32_t parity = UART_UARTDR_PE_BITS;
    static constexpr uint32_t brk = UART_UARTDR_BE_BITS;
    static constexpr uint32_t overrun = UART_UARTDR_OE_BITS;
    static constexpr uint32_t dropped = frame | parity | brk;
};

/// The FIFO trigger levels (UARTIFLS): the receive interrupt at or
/// above, the transmit interrupt at or below, this fraction of 32.
enum class UartFifoLevel : uint8_t { eighth = 0, quarter = 1, half = 2, three_quarters = 3, seven_eighths = 4 };

// ---- the resource ----------------------------------------------------------------

template <uint8_t n>
struct Pl011 {
    static_assert(n < 2, "the RP2040 has UART0 and UART1");
    Pl011() = delete;

    static constexpr uint8_t index = n;
    static constexpr uint32_t reset_bit = n == 0 ? ResetBlock::uart0 : ResetBlock::uart1;
    static constexpr uint8_t fifo_depth = 32;

    static UART0_Type& regs() { return *(n == 0 ? UART0 : UART1); }
    static constexpr IRQn_Type irq() { return n == 0 ? UART0_IRQ_IRQn : UART1_IRQ_IRQn; }

    /// The block from its reset state (2.14): held, released, ready.
    static bool reset() { return Resets::cycle(reset_bit); }
    static bool released() { return Resets::released(reset_bit); }
    /// Into reset and gone.
    static void hold() { Resets::hold(reset_bit); }

    static bool enabled() { return (regs().UARTCR & UART_UARTCR_UARTEN_BITS) != 0u; }
    /// UARTEN with both directions (TXE, RXE); off clears the three and
    /// nothing else of UARTCR (the loop-back bit stays what it was).
    static void enable(bool on) {
        constexpr uint32_t bits = UART_UARTCR_UARTEN_BITS | UART_UARTCR_TXE_BITS | UART_UARTCR_RXE_BITS;
        if (on) { hw_set(regs().UARTCR, bits); } else { hw_clear(regs().UARTCR, bits); }
    }

    /// The loop-back (UARTCR.LBE): the transmitter fed straight into the
    /// receiver, the RX pad ignored - a self-test with no wire. Refused
    /// (false, nothing written) while the UART is enabled, as every
    /// UARTCR change but the enable itself.
    static bool loopback(bool on) {
        if (enabled()) {
            return false;
        }
        if (on) { hw_set(regs().UARTCR, UART_UARTCR_LBE_BITS); }
        else { hw_clear(regs().UARTCR, UART_UARTCR_LBE_BITS); }
        return true;
    }
    static bool loopback() { return (regs().UARTCR & UART_UARTCR_LBE_BITS) != 0u; }

    /// The break (UARTLCR_H.BRK): TX held low after the current frame
    /// until cleared - a break longer than a frame is what a receiver
    /// reports as BE. Live: this one bit of LCR_H is meant to change
    /// under a running transmitter.
    static void break_send(bool on) {
        if (on) { hw_set(regs().UARTLCR_H, UART_UARTLCR_H_BRK_BITS); }
        else { hw_clear(regs().UARTLCR_H, UART_UARTLCR_H_BRK_BITS); }
    }
    static bool fifos_enabled() { return (regs().UARTLCR_H & UART_UARTLCR_H_FEN_BITS) != 0u; }

    /// The line control: frame format and the FIFO enable, one LCR_H
    /// write - which also latches whatever IBRD/FBRD hold. Refused (false,
    /// nothing written) while the UART is enabled.
    static bool line_control(const UartFormat& f, bool fifos) {
        if (enabled() || !uart_format_valid(f)) {
            return false;
        }
        uint32_t v = (static_cast<uint32_t>(f.bits) - 5u) << UART_UARTLCR_H_WLEN_LSB;
        if (f.stop_bits == 2u) {
            v |= UART_UARTLCR_H_STP2_BITS;
        }
        if (f.parity != UartParity::none) {
            v |= UART_UARTLCR_H_PEN_BITS;
            if (f.parity == UartParity::even) {
                v |= UART_UARTLCR_H_EPS_BITS;
            }
        }
        if (fifos) {
            v |= UART_UARTLCR_H_FEN_BITS;
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
        return {.integer = static_cast<uint16_t>(regs().UARTIBRD & UART_UARTIBRD_BAUD_DIVINT_BITS),
                .fraction = static_cast<uint8_t>(regs().UARTFBRD & UART_UARTFBRD_BAUD_DIVFRAC_BITS)};
    }

    /// The FIFO trigger levels.
    static void fifo_levels(UartFifoLevel rx, UartFifoLevel tx) {
        regs().UARTIFLS = (static_cast<uint32_t>(rx) << UART_UARTIFLS_RXIFLSEL_LSB) |
                          (static_cast<uint32_t>(tx) << UART_UARTIFLS_TXIFLSEL_LSB);
    }
    static UartFifoLevel rx_fifo_level() {
        return static_cast<UartFifoLevel>((regs().UARTIFLS & UART_UARTIFLS_RXIFLSEL_BITS) >>
                                          UART_UARTIFLS_RXIFLSEL_LSB);
    }
    static UartFifoLevel tx_fifo_level() {
        return static_cast<UartFifoLevel>((regs().UARTIFLS & UART_UARTIFLS_TXIFLSEL_BITS) >>
                                          UART_UARTIFLS_TXIFLSEL_LSB);
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

    /// Interrupt mask (UARTIMSC), through the atomic aliases: a
    /// handler and the loop can each touch their own bits.
    static void interrupts(uint32_t mask, bool on) {
        if (on) { hw_set(regs().UARTIMSC, mask); } else { hw_clear(regs().UARTIMSC, mask); }
    }
    static uint32_t interrupts() { return regs().UARTIMSC; }
    /// Masked status (UARTMIS): what is both raised and enabled.
    static uint32_t pending() { return regs().UARTMIS; }
    static uint32_t raw_pending() { return regs().UARTRIS; }
    static void clear_pending(uint32_t mask) { regs().UARTICR = mask; }

    /// The DMA request enables (UARTDMACR), for the engines' day.
    static void dma_requests(bool tx, bool rx) {
        regs().UARTDMACR = (tx ? UART_UARTDMACR_TXDMAE_BITS : 0u) |
                           (rx ? UART_UARTDMACR_RXDMAE_BITS : 0u);
    }
};

// ---- the task --------------------------------------------------------------------

/**
 * The interrupt-driven byte transport over Pl011<n> (the file header).
 *
 *   constexpr brio::UartPins console_pins{
 *       .tx = {0, brio::PinFunction::uart},
 *       .rx = {1, brio::PinFunction::uart},
 *   };
 *   using Serial = brio::Uart<0, console_pins>;
 *   constexpr Serial serial;                    // tag object (no state)
 *
 *   extern "C" void isr_uart0() { (void)Serial::isr(); }
 *
 *   int main() {
 *       SysClock::init();
 *       Serial::init(clock, 115200);
 *       brio::enable_interrupts();
 *       brio::print(serial, "hello", brio::crlf);
 *   }
 *
 * TX policy: write_byte() has TRY semantics (false when the TX ring is
 * full, nothing counted - the caller decides whether to retry, drop or
 * block; print.hpp blocks). RX overflow (ring full, byte lost) IS
 * counted, as are the hardware error flags.
 *
 * Ring sizes: this core reads a word atomically (atomic_width 4), so
 * util/ring.hpp takes its lock-free path at every size and a larger
 * ring costs only RAM. 64/256 are console-class defaults, not a ceiling.
 */
template <uint8_t n, UartPins pins, uint32_t rx_size = 64, uint32_t tx_size = 256,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
class Uart {
    using U = Pl011<n>;

    static_assert(uart_pins_valid(n, pins),
                  "these pins cannot carry this UART: under function 2 UART0 transmits "
                  "on GPIO 0, 12, 16 or 28 and receives on 1, 13, 17 or 29, UART1 "
                  "transmits on 4, 8, 20 or 24 and receives on 5, 9, 21 or 25 "
                  "(datasheet 2.19.2, table 279)");
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type");
    static_assert(!TxEngine::present && !RxEngine::present,
                  "the DMA engines of this stratum arrive with rp2040/dma.hpp; the slots "
                  "take NoDmaEngine until then");
    static_assert(uart_engines_distinct<TxEngine, RxEngine>(),
                  "the transmit and receive engines must use DIFFERENT DMA channels");

    using TxPin = Pin<pins.tx.pin>;
    using RxPin = Pin<pins.rx.pin>;

    // One ring pair per instantiation (static inline -> .bss, no ctor).
    static inline Ring<uint8_t, rx_size, Rp2040Platform<>> m_rx{};
    static inline Ring<uint8_t, tx_size, Rp2040Platform<>> m_tx{};

    // Error counters, written in the handler, read from the main loop.
    // A byte moves in one access on this core; they wrap at 255. Written
    // as `x = x + 1` because compound ops on volatile are deprecated in
    // C++20.
    static inline volatile uint8_t m_rx_overruns = 0;   // RX ring full, byte lost
    static inline volatile uint8_t m_frame_errors = 0;  // FE: byte dropped
    static inline volatile uint8_t m_parity_errors = 0; // PE: byte dropped
    static inline volatile uint8_t m_break_errors = 0;  // BE: a break, dropped
    static inline volatile uint8_t m_hw_overruns = 0;   // OE: the FIFO was full when a frame landed (UARTRSR)
    static inline uint32_t m_baud = 0;                  // for rebase()

public:
    /// Instances are empty tags for concept-based call sites (print(serial, ...)).
    constexpr Uart() = default;

    /// The resource underneath.
    using Resource = U;

    static constexpr bool has_tx_engine = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;

    // ---- lifecycle --------------------------------------------------------

    /// Bring the instance up: the block out of reset, the divisor and
    /// the line control, the FIFOs at their levels, the receive
    /// interrupts, the pads, the NVIC line.
    ///
    /// Call AFTER the main clock is set up and before interrupts are
    /// enabled globally; `clock` is the app's brio::Clock tag
    /// (rp2040/clock.hpp), so the divisor comes from Clock::pclk_hz -
    /// clk_peri - and never from a second statement of the rate.
    ///
    /// False when the rate cannot be produced at this clock or the
    /// block did not come out of reset - the caller then knows the
    /// transport is NOT up, rather than printing into a ring nothing
    /// will drain.
    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, const UartFormat& format = {}) {
        static_assert(clock_follows<Clock, Uart>(),
                      "this Uart is initialized with a DynamicClock that does not "
                      "list it among its Users: it would keep the old baud after "
                      "a clock change");
        const std::optional<UartDivisor> d = uart_divisor(Clock::pclk_hz, baud);
        (void)clock;
        if (!d || !uart_format_valid(format)) {
            return false;
        }

        // init() STARTS the transport: nothing a previous life left in
        // the rings or the counters is this one's traffic. The handler
        // is the rings' other party, so its line goes down first -
        // Ring::clear() is the one verb that is not concurrent.
        Nvic::disable(U::irq());
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
        U::interrupts(UartInterrupt::rx | UartInterrupt::rx_timeout, true);
        U::enable(true);

        // The pads go to the UART only now, with the transmitter
        // already enabled and its line idling high: handing the pad
        // over first would show whatever a disabled block drives. RX
        // gets a pull-up so an unconnected line reads idle rather than
        // a break (the pad's reset pull is DOWN).
        TxPin::function(PinFunction::uart);
        RxPin::function(PinFunction::uart, {.pull = PinPull::up});

        Nvic::enable(U::irq());
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
        U::enable(false);
        (void)U::divisor(*d);
        U::enable(true);
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
        U::enable(false);
        (void)U::divisor(*d);
        U::enable(true);
        m_baud = baud;
        return true;
    }

    /// The slowest clk_peri that can still produce `baud`.
    /// Change the frame format under the running port, once the TX side
    /// is idle; false and nothing written for a format the block has
    /// not got. Main context only.
    static bool set_format(const UartFormat& format) {
        if (!uart_format_valid(format)) {
            return false;
        }
        drain();
        U::enable(false);
        const bool ok = U::line_control(format, true);
        U::enable(true);
        return ok;
    }

    /// The loop-back on or off under the running port: the transmitter
    /// into the receiver, the RX pad ignored while on. Main context.
    static bool loopback(bool on) {
        drain();
        U::enable(false);
        const bool ok = U::loopback(on);
        U::enable(true);
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
        Nvic::disable(U::irq());
        U::interrupts(UartInterrupt::all, false);
        U::enable(false);
        TxPin::release();
        RxPin::release();
        U::hold();
    }

    // ---- the ISR body -----------------------------------------------------

    /// The instance's ONE interrupt body - call from isr_uart0() /
    /// isr_uart1().
    ///
    /// Serves what UARTMIS reports and, whatever it reports, moves the
    /// transmit ring into the FIFO: the entry may be the NVIC pend a
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
        if ((active & (UartInterrupt::rx | UartInterrupt::rx_timeout)) != 0u) {
            edge = receive();
        }
        feed();
        return edge;
    }

    // ---- byte transport (satisfies ByteSink / ByteSource) -----------------

    /// Try to queue one byte for transmission; false when the TX ring is
    /// full. The line is PENDED in the NVIC, never written to from here:
    /// the handler is the FIFO's one feeder.
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            Nvic::set_pending(U::irq());   // a refused byte still nudges: see below
            return false;
        }
        Nvic::set_pending(U::irq());
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
        Nvic::set_pending(U::irq());
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
    /// Nothing queued, nothing in the FIFO, nothing in the shifter.
    static bool tx_idle() { return m_tx.empty() && !U::busy(); }

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
        // bit of a FIFO entry is a LIVE condition ("cleared once there is
        // an empty space in the FIFO", 4.2.8) - the first read makes the
        // space, so the entries never carry it (measured); the status
        // register's OE is sticky until written, and one overrun event
        // is one count, however many frames it swallowed.
        if ((U::receive_status() & UART_UARTRSR_OE_BITS) != 0u) {
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
