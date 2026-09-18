/*
 * uart.hpp
 *
 * The UART (datasheet 12.1: two ARM PL011s, revision r1p5). The chapter's
 * driver is NOT here: the PL011 is a licensed design this chip shares
 * with others, so the resource and the byte transport are written once in
 * the IP stratum (brio/pl011/uart.hpp, docs/pl011/README.md) and this
 * file is what that file asks of a family -
 *
 *  - the pin table of 9.4 with its TWO function columns;
 *  - `Rp2350Pl011`, the CHIP TRAITS: the register block and where each
 *    instance's sits, the reset bits of 7.5, the interrupt lines of 3.2,
 *    the critical-section guard, the platform the rings are built on, the
 *    atomic set/clear aliases of 2.1.3, the DREQ numbers of 12.6.4.1, the
 *    pad type with the setup each direction wants, and WHICH RATE IS
 *    UARTCLK - clk_peri (12.1, figure 33), which rp2350/clock.hpp feeds
 *    from clk_sys undivided by default, so the divisor comes from
 *    Clock::pclk_hz and never from a second statement of the rate;
 *  - the PUBLIC NAMES: `Pl011<n>` the resource and `Uart<n, pins, ...>`
 *    the task, this chip's aliases of the two IP templates.
 *
 * THE PINS, AND THE COLUMN THE RP2040 HAD NOT. The pads march in groups
 * of four - TX, RX, CTS, RTS - and the groups cycle UART0, UART1, UART1,
 * UART0 and again, so GP0..GP3 are UART0's, GP4..GP7 UART1's, GP8..GP11
 * UART1's, GP12..GP15 UART0's, all the way to GP47 on the QFN-80. That
 * much is the RP2040's rule over a longer bank. What is new is that EVERY
 * GROUP'S FLOW-CONTROL PADS CARRY DATA TOO, under a second function code:
 * the CTS pad is also that instance's TX and the RTS pad also its RX,
 * under function 11 where the first two pads use function 2. So GP2/GP3
 * are a second UART0 pair, GP6/GP7 a second UART1 pair, and a QFN-80 chip
 * offers twelve transmit pads and twelve receive pads per instance
 * instead of the RP2040's four.
 *
 * WHICH COLUMN A PAD USES IS NOT A CHOICE: a pad carries a UART signal
 * under exactly one function code, and the pad number says which. That is
 * why `pad_function` here is a TYPE and not a number - the IP file calls
 * it "the function code that routes a UART to a pad" and keeps it OPAQUE,
 * which is exactly the room a chip with a column per pad needs. The tag
 * travels, `UartPad<pin>` resolves it against the table, and the IP file
 * is untouched.
 *
 * THE LINE. The eleven maskable sources of an instance combine into ONE
 * interrupt line (UART0_IRQ = 33, UART1_IRQ = 34, datasheet 3.2); the app
 * binds isr_uart0 / isr_uart1 to the instance's isr() body, and those
 * names are bound on BOTH architectures - a Cortex-M vector table on one
 * half, Hazard3's own dispatch on the other - because the interrupt
 * numbering is shared (3.8.4.2). Which is also why `Irq` below is the
 * controller core.hpp names and never an NVIC.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

#include "pl011/uart.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma_engine.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/platform.hpp"
#include "rp2350/resets.hpp"

namespace brio {

// ---- the pads --------------------------------------------------------------------

/// The two pads of an asynchronous link, each named with its function -
/// which on this chip is the pad's own column and not one constant
/// (`uart_pad_function`).
struct UartPins {
    PinSel tx;
    PinSel rx;
};

/// Which instance the four pads of `pin`'s group belong to (9.4): the
/// groups cycle UART0, UART1, UART1, UART0, ... over the whole bank.
constexpr uint8_t uart_pad_instance(uint8_t pin) {
    const uint8_t group = pin / 4u;
    return static_cast<uint8_t>(((group + 1u) / 2u) % 2u);
}

/// WHICH FUNCTION routes a UART signal to `pin`: function 2 on the first
/// two pads of a group, function 11 on the other two - the column the
/// RP2040 had not, which turns every flow-control pad into a second data
/// pad of the same instance.
constexpr PinFunction uart_pad_function(uint8_t pin) {
    return (pin % 4u) < 2u ? PinFunction::uart : PinFunction::uart_alt;
}

/// Is `pin` instance n's TRANSMIT pad? The first pad of a group carries
/// TX under function 2, the third under function 11.
constexpr bool uart_tx_pin(uint8_t n, uint8_t pin) {
    if (pin >= gpio_count_max) {
        return false;
    }
    const uint8_t within = pin % 4u;
    return (within == 0u || within == 2u) && uart_pad_instance(pin) == n;
}

/// And its RECEIVE pad: the second pad of a group under function 2, the
/// fourth under function 11.
constexpr bool uart_rx_pin(uint8_t n, uint8_t pin) {
    if (pin >= gpio_count_max) {
        return false;
    }
    const uint8_t within = pin % 4u;
    return (within == 1u || within == 3u) && uart_pad_instance(pin) == n;
}

/// A legal pin set for instance n: a transmit pad and a receive pad the
/// table gives that instance, EACH NAMED UNDER ITS OWN COLUMN - naming
/// GP2 under function 2 asks for UART0's CTS and not its transmitter, so
/// it is refused here rather than routed to the wrong signal.
///
/// The PACKAGE is not asked here: every pin's registers exist on both
/// packages and only the bond wire does not, so a pad the QFN-60 has not
/// got is refused one level down, by `Pin<n>`'s own static_assert, which
/// says so in those words.
constexpr bool uart_pins_valid(uint8_t n, const UartPins& p) {
    return uart_tx_pin(n, p.tx.pin) && uart_rx_pin(n, p.rx.pin) &&
           p.tx.function == uart_pad_function(p.tx.pin) &&
           p.rx.function == uart_pad_function(p.rx.pin);
}

/// WHAT `Rp2350Pl011::pad_function` IS. The IP file hands this value to a
/// pad and never looks inside it ("the function code that routes a UART
/// to a pad, opaque to this file"); on the RP2040 the answer really was
/// one code, and here it is a COLUMN of the table whose number depends on
/// the pad. So the chip's answer is this tag - "the column that carries a
/// UART signal" - and `UartPad<pin>` below turns it into the one code
/// that routes a UART to that pad.
struct UartColumn {};

/// A UART pad as a type: the two verbs the IP driver calls on one, over
/// the pin chapter's own. Handing it over resolves the column against the
/// pad number; taking it back is the pin's ordinary release, which leaves
/// the pad ISOLATED again with its pull-down, this chip's reset state.
template <uint8_t pin>
struct UartPad {
    static bool function(UartColumn, const PinConfig& cfg) {
        return Pin<pin>::function(uart_pad_function(pin), cfg);
    }
    static bool release() { return Pin<pin>::release(); }
};

// ---- what this chip tells the PL011 driver ----------------------------------------

/**
 * The chip traits brio/pl011/uart.hpp is instantiated with: every fact
 * about the PL011 that is the RP2350's rather than ARM's, and nothing
 * else. Nothing above this file ever names it.
 */
struct Rp2350Pl011 {
    Rp2350Pl011() = delete;

    using Regs = UART0_Type;
    /// What the controller calls a line. `Interrupts` below is the
    /// controller itself, which on this chip is `brio::Irq` - an NVIC
    /// under one architecture and Hazard3's own under the other, named
    /// once in core.hpp. The qualified spelling is what keeps this
    /// member from shadowing it.
    using Irq = IRQn_Type;
    using Interrupts = ::brio::Irq;
    using Guard = InterruptGuard;
    using Platform = Rp2350Platform<>;
    using Pins = UartPins;
    using DmaRequest = Dreq;

    /// UART0 and UART1, both with 32-deep FIFOs (12.1: 32x8 transmit,
    /// 32x12 receive - the same synthesis as the RP2040's).
    static constexpr uint8_t instances = 2;
    static constexpr uint8_t fifo_depth = 32;

    template <uint8_t i>
    static Regs& regs() { return *(i == 0 ? UART0 : UART1); }
    template <uint8_t i>
    static constexpr Irq irq() { return i == 0 ? UART0_IRQ_IRQn : UART1_IRQ_IRQn; }

    /// The instance's bit in the subsystem reset controller (7.5).
    template <uint8_t i>
    static constexpr uint32_t reset_bit() { return i == 0 ? ResetBlock::uart0 : ResetBlock::uart1; }

    template <uint8_t i>
    static bool reset() { return Resets::cycle(reset_bit<i>()); }
    template <uint8_t i>
    static bool released() { return Resets::released(reset_bit<i>()); }
    template <uint8_t i>
    static void hold() { Resets::hold(reset_bit<i>()); }

    /// 12.6.4.1: the two data requests of an instance. NOT the RP2040's
    /// numbers - a third PIO moved every row above it (dma_engine.hpp).
    template <uint8_t i>
    static constexpr DmaRequest tx_request() { return i == 0 ? Dreq::uart0_tx : Dreq::uart1_tx; }
    template <uint8_t i>
    static constexpr DmaRequest rx_request() { return i == 0 ? Dreq::uart0_rx : Dreq::uart1_rx; }

    /// The atomic register aliases of 2.1.3: one bus write, no
    /// read-modify-write.
    static void set_bits(volatile uint32_t& reg, uint32_t bits) { hw_set(reg, bits); }
    static void clear_bits(volatile uint32_t& reg, uint32_t bits) { hw_clear(reg, bits); }

    /// The pin table above, and the two halves of a pin set.
    static constexpr bool pins_valid(uint8_t n, const Pins& p) { return uart_pins_valid(n, p); }
    static constexpr uint8_t tx_pad(const Pins& p) { return p.tx.pin; }
    static constexpr uint8_t rx_pad(const Pins& p) { return p.rx.pin; }

    /// The pad as a type, the COLUMN that carries a UART signal (the tag
    /// above: on this chip the code is the pad's), and the electrical
    /// setup each direction wants - the receive one WITH ITS PULL-UP,
    /// because this chip's pads come up pulled DOWN and a receiver
    /// enabled over a low line takes a break (the IP file's init() says
    /// why that matters). Every configuring verb of pin.hpp drops the
    /// ISOLATION LATCH as it writes, which is what makes a pad of this
    /// chip answer at all.
    template <uint8_t pin>
    using Pad = UartPad<pin>;
    static constexpr UartColumn pad_function{};
    static constexpr PinConfig tx_pad_config() { return {}; }
    static constexpr PinConfig rx_pad_config() { return {.pull = PinPull::up}; }

    /// Two engines of one transport must not share a channel: on this
    /// chip a request is a field any channel takes, so the CHANNEL is the
    /// identity (rp2350/dma_engine.hpp).
    template <typename Tx, typename Rx>
    static constexpr bool engines_distinct() { return uart_engines_distinct<Tx, Rx>(); }

    /// UARTCLK is clk_peri (12.1) - `Clock::pclk_hz`, the one truth about
    /// the rate the divisor is computed from.
    template <typename Clock>
    static constexpr uint32_t uartclk_hz(Clock) { return Clock::pclk_hz; }
};

static_assert(Pl011Chip<Rp2350Pl011>);

// ---- the public names --------------------------------------------------------------

/// The resource: which instance, where its registers are, its reset bit,
/// its interrupt line, and the whole of the PL011's programmer's model.
template <uint8_t n>
using Pl011 = Pl011Uart<Rp2350Pl011, n>;

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
 * The same source builds for both architectures: `isr_uart0` is a
 * Cortex-M vector on one half and a Hazard3 dispatch entry on the other,
 * under one name.
 *
 * A pad of the second column is named under that column, and nothing else
 * changes:
 *
 *   constexpr brio::UartPins alt_pins{
 *       .tx = {2, brio::PinFunction::uart_alt},
 *       .rx = {3, brio::PinFunction::uart_alt},
 *   };
 *
 * Ring sizes: this core reads a word atomically (atomic_width 4), so
 * util/ring.hpp takes its lock-free path at every size and a larger ring
 * costs only RAM. 64/256 are console-class defaults, not a ceiling.
 */
template <uint8_t n, UartPins pins, uint32_t rx_size = 64, uint32_t tx_size = 256,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
using Uart = Pl011Transport<Rp2350Pl011, n, pins, rx_size, tx_size, TxEngine, RxEngine>;

} // namespace brio
