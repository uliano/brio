/*
 * usart.hpp
 *
 * The serial port of the CH32V00x, as the interrupt-driven byte
 * transport the rest of brio talks to: `Uart<1>` satisfies
 * util/stream.hpp's ByteTransport, so print(), SerialPort and the
 * console protocol sit on it unchanged - the same surface the other
 * three targets expose, which is the whole point of the layering.
 *
 * TWO RINGS AND TWO FLAGS. Bytes leave through a TX ring drained by
 * the TXE interrupt (write_byte() re-arms TXEIE; the ISR disarms it
 * when the ring runs dry, because TXE stands for ever while the
 * transmitter is idle and would re-enter the handler without end), and
 * arrive into an RX ring filled by RXNE. isr() returns the "RX went
 * non-empty" EDGE that util/serial_port.hpp wants to post on - one
 * event per idle-to-busy transition, not one per byte.
 *
 * THE BAUD DIVISOR IS THE WHOLE REGISTER. BRR counts the peripheral
 * clock's periods per bit in sixteenths (RM 16.6.3: a 12-bit integer
 * part and a 4-bit fraction), so the value to store is simply
 * pclk / baud - no shifting, no separate fields - and actual_baud()
 * inverts it, which is how a program can report the error it is
 * actually running with.
 *
 * ERRORS ARE READ THEN CLEARED. Framing, noise, parity and hardware
 * overrun stand in STATR and are cleared by reading STATR and then
 * DATAR, in that order (RM 16.6.1) - so the handler reads the status
 * once, decides from that copy, and lets the DATAR read do the
 * clearing. A byte that arrived with an error is DROPPED rather than
 * pushed: a corrupted byte in a line assembler is worse than a gap,
 * and the counter says it happened.
 *
 * NOT COVERED YET:
 *  - USART2 (CH32V005/006/007 only): its pads want the AFIO remap
 *    registers this stratum does not touch yet, so it is refused at
 *    compile time rather than half-built. It arrives with the first
 *    program that needs a second port.
 *  - the alternate-function REMAPS themselves (RM 7.3, table 7-11 for
 *    USART2), so USART1 is reachable on its default pads only.
 *  - the chapter's other personalities - synchronous mode, half
 *    duplex, LIN, IrDA, smartcard, the hardware flow control pins, DMA
 *    - each of which is a task over this same resource on the other
 *    targets, and each of which arrives with a bench that needs it.
 */

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"
#include "util/stream.hpp"

namespace brio {

/// The default pads of an instance (the datasheet's pin table; no
/// remap is applied by this stratum yet).
struct UsartPads {
    char tx_port;
    uint8_t tx_pin;
    char rx_port;
    uint8_t rx_pin;
};

/// USART1: TX on PD5, RX on PD6 - the pads the WCH-Link's own serial
/// is wired to on the bench board.
constexpr UsartPads usart_pads_for(uint8_t instance) {
    return instance == 1 ? UsartPads{'D', 5, 'D', 6} : UsartPads{'\0', 0, '\0', 0};
}

constexpr uint32_t usart_base_for(uint8_t instance) {
    return instance == 1 ? pb2_base + 0x3800 : 0;
}

constexpr uint32_t usart_clock_for(uint8_t instance) {
    return instance == 1 ? rcc_pb2_usart1 : 0;
}

constexpr Irq usart_irq_for(uint8_t instance) {
    return instance == 1 ? Irq::usart1 : Irq::usart1;
}

/**
 * The interrupt-driven serial port.
 *
 *   using Serial = brio::Uart<1>;
 *   constexpr Serial serial;                 // tag for print(serial, ...)
 *   Serial::init(clock, 115200);
 *   extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { Serial::isr(); }
 *
 * `P` is the platform, which the rings need to know whether an index
 * can be shared with a handler bare (atomic_width) or wants a guard.
 */
template <uint8_t instance, typename P, uint16_t rx_size = 64, uint16_t tx_size = 64>
struct Uart {
    static_assert(usart_base_for(instance) != 0,
                  "brio Uart: only USART1 is implemented on the CH32V00x today - "
                  "USART2 exists on the CH32V005/006/007 and needs the AFIO remaps "
                  "first (see this file's 'Not covered yet')");

    Uart() = default;   // a tag instance: constexpr Uart<1, P> serial;

    static constexpr uint8_t number = instance;
    static constexpr UsartPads pads = usart_pads_for(instance);

    using Tx = Pin<pads.tx_port, pads.tx_pin>;
    using Rx = Pin<pads.rx_port, pads.rx_pin>;

    static UsartRegs& regs() {
        return *reinterpret_cast<UsartRegs*>(usart_base_for(instance));
    }

    /**
     * Open the port at `baud`, 8N1, with the receive interrupt armed.
     *
     * Returns false when the divisor comes out below the smallest the
     * chapter allows (16, i.e. one whole peripheral-clock period per
     * bit sixteenth): a baud rate this clock cannot serve is a fact the
     * caller must see rather than a port that mangles every frame.
     */
    template <typename C>
    static bool init(C clock, uint32_t baud) {
        static_assert(clock_follows<C, Uart>(),
                      "brio Uart: the baud divisor is derived from the peripheral "
                      "clock, so a dynamic clock must list this port among the users "
                      "it rebases");

        const uint32_t pclk = clock_hz(clock);
        const uint32_t brr = divisor_for(pclk, baud);
        if (brr < 16u || brr > 0xFFFFu) {
            return false;
        }

        rcc()->PB2PCENR |= usart_clock_for(instance);

        Tx::function();          // alternate function, push-pull
        Rx::input();             // floating: the peer drives it

        regs().CTLR1 = 0;        // configure with the port disabled
        regs().CTLR2 = 0;        // one stop bit
        regs().CTLR3 = 0;        // no flow control, no DMA
        regs().BRR = static_cast<uint16_t>(brr);
        regs().CTLR1 = usart_ue | usart_te | usart_re | usart_rxneie;
        m_baud = baud;

        m_tx.clear();
        m_rx.clear();
        clear_errors();

        Pfic::enable(usart_irq_for(instance));
        return true;
    }

    /// The divisor this clock and baud ask for: pclk/baud in sixteenths,
    /// rounded to nearest so the error is halved.
    static constexpr uint32_t divisor_for(uint32_t pclk, uint32_t baud) {
        return baud == 0u ? 0u : (pclk + baud / 2u) / baud;
    }

    /// The baud the port is ACTUALLY running at, from the divisor in the
    /// register - what a program reports instead of what it asked for.
    static uint32_t actual_baud(uint32_t pclk) {
        const uint32_t brr = regs().BRR;
        return brr == 0u ? 0u : pclk / brr;
    }

    /**
     * The port's whole interrupt body - bind the USART1 vector to it.
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

    // ---- introspection ----------------------------------------------------

    static auto rx_pending() { return m_rx.count(); }
    static bool tx_idle() { return m_tx.empty(); }

    static uint16_t rx_overruns() { return m_rx_overruns; }
    static uint16_t frame_errors() { return m_frame_errors; }
    static uint16_t parity_errors() { return m_parity_errors; }
    static uint16_t noise_errors() { return m_noise_errors; }
    static uint16_t hw_overruns() { return m_hw_overruns; }

    static void clear_errors() {
        m_rx_overruns = 0;
        m_frame_errors = 0;
        m_parity_errors = 0;
        m_noise_errors = 0;
        m_hw_overruns = 0;
    }

    /// Follow a clock that changed rate. Drains what is in flight first:
    /// a byte half out at the old rate would finish at the new one.
    static void rebase(uint32_t hz) {
        while (!m_tx.empty()) { }
        const uint32_t baud = actual_baud_cached();
        const uint32_t brr = divisor_for(hz, baud);
        if (brr >= 16u && brr <= 0xFFFFu) {
            regs().BRR = static_cast<uint16_t>(brr);
        }
    }

private:
    /// Saturating: a counter that wrapped would understate the damage.
    static void bump(uint16_t& counter) {
        if (counter != 0xFFFFu) {
            ++counter;
        }
    }

    static uint32_t actual_baud_cached() { return m_baud; }

    static inline Ring<uint8_t, rx_size, P> m_rx;
    static inline Ring<uint8_t, tx_size, P> m_tx;
    static inline uint32_t m_baud = 0;

    static inline uint16_t m_rx_overruns = 0;
    static inline uint16_t m_frame_errors = 0;
    static inline uint16_t m_parity_errors = 0;
    static inline uint16_t m_noise_errors = 0;
    static inline uint16_t m_hw_overruns = 0;
};

} // namespace brio
