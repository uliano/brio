/*
 * uart.hpp
 *
 * The UART (datasheet 4.2: two ARM PL011s). The chapter's driver is NOT
 * here: the PL011 is a licensed design this chip shares with others, so
 * the resource and the byte transport are written once in the IP stratum
 * (brio/pl011/uart.hpp, docs/pl011/README.md) and this file is what that
 * file asks of a family -
 *
 *  - the pin table of 2.19.2 (table 279) with its function code;
 *  - `Rp2040Pl011`, the CHIP TRAITS: the register block and where each
 *    instance's sits, the reset bits of 2.14, the NVIC lines of 2.3.2,
 *    the critical-section guard, the platform the rings are built on,
 *    the atomic set/clear aliases of 2.1.2, the DREQ numbers of table
 *    119, the pad type with the function code that routes a UART to it
 *    and the setup each direction wants, and WHICH RATE IS UARTCLK -
 *    clk_peri (2.15.3.1), which rp2040/clock.hpp feeds from clk_sys
 *    undivided, so the divisor comes from Clock::pclk_hz and never from
 *    a second statement of the rate;
 *  - the PUBLIC NAMES: `Pl011<n>` the resource and `Uart<n, pins, ...>`
 *    the task, this chip's aliases of the two IP templates.
 *
 * THE PINS. UART0 and UART1 each reach four TX pads and four RX pads
 * under function 2 (2.19.2, table 279), fixed per instance: UART0 TX on
 * GPIO 0, 12, 16, 28 and RX on 1, 13, 17, 29; UART1 TX on 4, 8, 20, 24
 * and RX on 5, 9, 21, 25. `uart_pins_valid` is that table, and a pin
 * set that is not in it does not compile. CTS/RTS pads exist (the next
 * pins of each group) and hardware flow control is a gap
 * (docs/rp2040/uart.md).
 *
 * THE LINE. The eleven maskable sources of an instance combine into ONE
 * NVIC line (UART0_IRQ, UART1_IRQ, table 2.3.2); the app binds
 * isr_uart0 / isr_uart1 to the instance's isr() body.
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
    /// direction wants - the receive one WITH ITS PULL-UP, because this
    /// chip's pads come up pulled DOWN and a receiver enabled over a low
    /// line takes a break (the IP file's init() says why that matters).
    template <uint8_t pin>
    using Pad = Pin<pin>;
    static constexpr PinFunction pad_function = PinFunction::uart;
    static constexpr PinConfig tx_pad_config() { return {}; }
    static constexpr PinConfig rx_pad_config() { return {.pull = PinPull::up}; }

    /// Two engines of one transport must not share a channel: on this
    /// chip a request is a field any channel takes, so the CHANNEL is
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
 * Ring sizes: this core reads a word atomically (atomic_width 4), so
 * util/ring.hpp takes its lock-free path at every size and a larger
 * ring costs only RAM. 64/256 are console-class defaults, not a ceiling.
 * The two engine slots take rp2040/dma.hpp's DmaTxEngine / DmaRxEngine.
 */
template <uint8_t n, UartPins pins, uint32_t rx_size = 64, uint32_t tx_size = 256,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
using Uart = Pl011Transport<Rp2040Pl011, n, pins, rx_size, tx_size, TxEngine, RxEngine>;

} // namespace brio
