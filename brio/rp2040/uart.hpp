/*
 * uart.hpp
 *
 * The RP2040's serial port for a program: the console a program prints
 * to, a line-oriented link, a byte transport to another chip.
 * `Uart<n, pins, ...>` is the task (in brio, a driver layer with state
 * of its own, served from its peripheral's interrupt, that an active
 * object or print() calls; an active object is a component of a brio
 * program with its own event queue, served one event at a time by the
 * kernel): a byte transport with a transmit ring and a receive ring,
 * the public surface every target's Uart has, which is what lets
 * util/serial_port.hpp and print() run here unchanged. `Pl011<n>` is
 * the resource under it, one function per register fact. The smallest
 * use, the console on UART0 (written out at `Uart` below, as
 * docs/rp2040/uart.md spells it): the two pads named with function 2,
 * `using Serial = brio::Uart<0, console_pins>`, the program's
 * `isr_uart0` calling `Serial::isr()`, `Serial::init(clock, 115200)`
 * after `SysClock::init()`, then print().
 *
 * Who calls what. init() and release() are setup verbs: main(), after
 * the clock task's init() (the divisor is computed from the rate the
 * clock type claims, not read from the hardware, and clk_peri, the
 * UARTs' clock, stays off until that init() enables it) and before
 * interrupts are enabled globally; init() answers false when the rate
 * cannot be produced at this clock, the format is refused
 * (uart_format_valid), the block did not come out of reset, or the
 * resource's divisor() or line_control() refused its write (both refuse
 * while the block is enabled).
 * isr() is the interrupt body, called by the program's handler for the
 * instance's vector and by nothing else; it returns true on the receive
 * ring's empty-to-non-empty edge, the edge a program posts RxActivity
 * on. dma_isr() is the DMA line's body when an engine fills a slot. The
 * transmit verbs (write_byte(), write_bulk(), and print() over them)
 * and the receive verbs (read_byte(), read_bulk(), read_span() with
 * consume(), harvest()) belong to the main context - setup, or an
 * active object's handler - and never to an interrupt handler: each
 * ring has one producer and one consumer, and the interrupt body is the
 * other end of both. Without engines nothing masks interrupts: the
 * interrupt enables go through the atomic set/clear aliases. With a
 * transmit engine the engine's claim is taken under the platform's
 * guard (interrupts masked for the test-and-set alone); harvest() runs
 * under the guard too. set_baud(), set_format(), loopback() and
 * rebase() wait, bounded, for the transmit ring and the shifter to
 * empty before they touch the line. No transmit or receive verb waits:
 * write_byte() answers false when the transmit ring is full,
 * write_bulk() returns how many bytes it took (short of the run when
 * the ring filled), and print() is the caller that spins until every
 * byte is taken. Storage: the rings are the task's own static arrays;
 * write_bulk() copies the caller's run (a buffer and its length),
 * which is free again when it returns; read_span() hands out the
 * ring's own bytes, valid until consume() releases them.
 *
 * The UART chapter's driver (datasheet 4.2: two ARM PL011s) is not here:
 * the PL011 is a licensed design this chip shares with others, so the
 * resource and the byte transport are written once in the IP stratum
 * (brio/pl011/uart.hpp, docs/pl011/README.md) and this file is what that
 * file asks of a family -
 *
 *  - the pin table of 2.19.2 (table 279) with its function code;
 *  - `Rp2040Pl011`, the chip traits: the register block and where each
 *    instance's sits, the reset bits of 2.14, the NVIC lines of 2.3.2,
 *    the critical-section guard, the platform the rings are built on,
 *    the atomic set/clear aliases of 2.1.2, the DREQ numbers of table
 *    119, the pad type with the function code that routes a UART to it
 *    and the setup each direction wants, and which rate is UARTCLK -
 *    clk_peri (2.15.3.1), which rp2040/clock.hpp feeds from clk_sys
 *    undivided by default or from the crystal under
 *    `PeriSource::crystal`, so the divisor comes from Clock::pclk_hz
 *    and never from a second statement of the rate;
 *  - the public names: `Pl011<n>` the resource and `Uart<n, pins, ...>`
 *    the task, this chip's aliases of the two IP templates.
 *
 * The pins. UART0 and UART1 each reach four TX pads and four RX pads
 * under function 2 (2.19.2, table 279), fixed per instance: UART0 TX on
 * GPIO 0, 12, 16, 28 and RX on 1, 13, 17, 29; UART1 TX on 4, 8, 20, 24
 * and RX on 5, 9, 21, 25. `uart_pins_valid` is that table, and a pin
 * set that is not in it does not compile. CTS/RTS pads exist (the next
 * pins of each group) and hardware flow control is a gap
 * (docs/rp2040/uart.md).
 *
 * The line. The eleven maskable sources of an instance combine into one
 * NVIC line (UART0_IRQ, UART1_IRQ, table 2.3.2); the app binds
 * isr_uart0 / isr_uart1 to the instance's isr() body. A vector left
 * unbound sends the first interrupt into the crt's default handler and
 * the program stops there (docs/rp2040/platform.md, the handler names).
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

#include "pl011/uart.hpp"
#include "rp2040/clock.hpp"
#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/platform.hpp"
#include "rp2040/resets.hpp"

namespace brio {

// ---- the pads --------------------------------------------------------------------

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

// ---- what this chip tells the PL011 driver ----------------------------------------

/**
 * The chip traits brio/pl011/uart.hpp is instantiated with: every fact
 * about the PL011 that is the RP2040's rather than ARM's, and nothing
 * else. Nothing above this file ever names it.
 */
struct Rp2040Pl011 {
    Rp2040Pl011() = delete;

    using Regs = UART0_Type;
    using Irq = IRQn_Type;
    using Interrupts = Nvic;
    using Guard = InterruptGuard;
    using Platform = Rp2040Platform<>;
    using Pins = UartPins;
    using DmaRequest = Dreq;

    /// UART0 and UART1, both with 32-deep FIFOs (4.2.2.4/5).
    static constexpr uint8_t instances = 2;
    static constexpr uint8_t fifo_depth = 32;
    /// write_bulk()'s memcpy crossover on this core (bench_rp2040 letter
    /// u, the Cortex-M0+ at 125 MHz from the XIP cache): the byte loop is
    /// 2 cycles and 9 a byte, the runtime's memcpy about 54 and under one
    /// a byte when the two ends share their word alignment - the loop
    /// still ahead at 8 bytes, memcpy at 12.
    static constexpr uint32_t copy_crossover = 12;

    template <uint8_t i>
    static Regs& regs() { return *(i == 0 ? UART0 : UART1); }
    template <uint8_t i>
    static constexpr Irq irq() { return i == 0 ? UART0_IRQ_IRQn : UART1_IRQ_IRQn; }

    /// The instance's bit in the subsystem reset controller (2.14).
    template <uint8_t i>
    static constexpr uint32_t reset_bit() { return i == 0 ? ResetBlock::uart0 : ResetBlock::uart1; }

    template <uint8_t i>
    static bool reset() { return Resets::cycle(reset_bit<i>()); }
    template <uint8_t i>
    static bool released() { return Resets::released(reset_bit<i>()); }
    template <uint8_t i>
    static void hold() { Resets::hold(reset_bit<i>()); }

    /// Table 119: the two data requests of an instance.
    template <uint8_t i>
    static constexpr DmaRequest tx_request() { return i == 0 ? Dreq::uart0_tx : Dreq::uart1_tx; }
    template <uint8_t i>
    static constexpr DmaRequest rx_request() { return i == 0 ? Dreq::uart0_rx : Dreq::uart1_rx; }

    /// The atomic register aliases of 2.1.2: one bus write, no
    /// read-modify-write.
    static void set_bits(volatile uint32_t& reg, uint32_t bits) { hw_set(reg, bits); }
    static void clear_bits(volatile uint32_t& reg, uint32_t bits) { hw_clear(reg, bits); }

    /// The pin table above, and the two halves of a pin set.
    static constexpr bool pins_valid(uint8_t n, const Pins& p) { return uart_pins_valid(n, p); }
    static constexpr uint8_t tx_pad(const Pins& p) { return p.tx.pin; }
    static constexpr uint8_t rx_pad(const Pins& p) { return p.rx.pin; }

    /// The pad as a type, the function code that routes a UART to it
    /// (2.19.2: F2 on every UART pin), and the electrical setup each
    /// direction wants - the receive one with its pull-up, because this
    /// chip's pads come up pulled down and a receiver enabled over a low
    /// line takes a break (the IP file's init() says why that matters).
    template <uint8_t pin>
    using Pad = Pin<pin>;
    static constexpr PinFunction pad_function = PinFunction::uart;
    static constexpr PinConfig tx_pad_config() { return {}; }
    static constexpr PinConfig rx_pad_config() { return {.pull = PinPull::up}; }

    /// Two engines of one transport must not share a channel: on this
    /// chip a request is a field any channel takes, so the channel is
    /// the identity (rp2040/dma_engine.hpp).
    template <typename Tx, typename Rx>
    static constexpr bool engines_distinct() { return uart_engines_distinct<Tx, Rx>(); }

    /// UARTCLK is clk_peri (2.15.3.1) - `Clock::pclk_hz`, the one truth
    /// about the rate the divisor is computed from.
    template <typename Clock>
    static constexpr uint32_t uartclk_hz(Clock) { return Clock::pclk_hz; }
};

static_assert(Pl011Chip<Rp2040Pl011>);

// ---- the public names --------------------------------------------------------------

/// The resource: which instance, where its registers are, its reset bit,
/// its NVIC line, and the whole of the PL011's programmer's model.
template <uint8_t n>
using Pl011 = Pl011Uart<Rp2040Pl011, n>;

/**
 * The task: the asynchronous byte transport over Pl011<n>.
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
 * Line by line: `console_pins` names the two pads with function 2, which
 * table 279 gives UART0 on GPIO 0 and 1 (uart_pins_valid refuses at
 * compile time a pad the table does not give the instance; any of UART0's
 * four TX pads pairs with any of its four RX pads); `Serial` is the
 * transport type, every other template argument at its default (the ring
 * sizes, no DMA engines, InFlash); `serial` is the empty tag object
 * print() takes to pick the port, the type's functions being all static;
 * the `extern "C"` function is the interrupt handler under the pico-sdk's
 * name for the vector; init() takes the clock task's tag and the baud
 * rate. Under the kernel the handler posts the edge isr() returns:
 * `if (Serial::isr()) { brio::post<SerialLines>(brio::RxActivity{}); }`,
 * where SerialLines is the program's active object that reads the lines.
 *
 * Ring sizes: this core reads a word atomically (atomic_width 4), so
 * util/ring.hpp takes its lock-free path at every size and a larger
 * ring costs only RAM. 64/256 are console-class defaults, not a ceiling.
 * The two engine slots take rp2040/dma.hpp's DmaTxEngine / DmaRxEngine,
 * on two different channels (engines_distinct): init() arms them, so
 * `Dma::init()` - the DMA block out of reset, once per program - comes
 * first, and the program binds the DMA line's vector to dma_isr().
 * `Placement`, the last parameter, is util/placement.hpp's tag: InRam
 * makes the bodies of isr() and dma_isr() one flattened instance each
 * in SRAM (docs/rp2040/platform.md, "A handler in SRAM").
 */
template <uint8_t n, UartPins pins, uint32_t rx_size = 64, uint32_t tx_size = 256,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine, typename Placement = InFlash>
using Uart = Pl011Transport<Rp2040Pl011, n, pins, rx_size, tx_size, TxEngine, RxEngine, Placement>;

} // namespace brio
