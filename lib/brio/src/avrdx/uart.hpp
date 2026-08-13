/*
 * uart.hpp
 *
 * Interrupt-driven USART byte transport for the AVR Dx families.
 * Rewritten 07/2026 from the AVR-Multislope object-based driver as a static
 * (monostate) class, following the framework decisions:
 *
 *  - all state is `static inline` (one per instantiation, .bss, no ctor
 *    running before main): hardware is touched ONLY by the explicit init(),
 *    called after the clock is up;
 *  - no ByteStream base class: the driver satisfies the brio::ByteSink /
 *    brio::ByteSource concepts (stream.hpp) with static try-semantics calls;
 *  - byte transport only - text formatting lives in print.hpp;
 *  - RX hardware error flags (frame / parity / buffer overflow) are read
 *    from RXDATAH and counted; corrupted bytes (FERR/PERR) are dropped.
 *
 * The empty default constructor is intentionally AVAILABLE: an instance
 * carries no state and acts as a zero-cost tag so call sites can read
 * naturally, e.g.
 *
 *   using Serial = brio::Uart<2, brio::Route::alt1>;
 *   constexpr Serial serial;                 // tag object (no state)
 *
 *   ISR(USART2_RXC_vect) { Serial::rxc(); }
 *   ISR(USART2_DRE_vect) { Serial::dre(); }
 *
 *   int main() {
 *       brio::init_clock_24mhz();
 *       Serial::init(460800);
 *       sei();
 *       brio::print(serial, "hello", brio::crlf);
 *       ...
 *   }
 *
 * TX policy: write_byte() has TRY semantics (false when the TX ring is
 * full, nothing counted - the caller decides whether to retry, drop or
 * block; print.hpp blocks). RX overflow (ring full, byte lost) IS counted,
 * as are the hardware error flags.
 */

#pragma once

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include "avrdx/ring.hpp"
#include "util/stream.hpp"

namespace brio {

/// USART pin routing (PORTMUX). alt1 = the ALT1 position of the instance.
enum class Route : uint8_t { def = 0, alt1 = 1 };

// Ring defaults sized for console-class traffic AND for cheap indices:
// both <= 256 keeps Ring's index_t at 8 bits (measured on write_byte: the
// 512-byte TX ring's 16-bit indices cost ~10 extra cycles per byte). RX 64
// absorbs ~1.4 ms of full-rate 460800 traffic - plenty against dispatches
// that last microseconds. Streaming apps may still ask for more.
template <int usart_num, Route route = Route::def,
          int rx_size = 64, int tx_size = 256>
class Uart {
    static_assert(usart_num >= 0 && usart_num <= 5,
                  "usart_num must be 0..5");

    // One ring pair per instantiation (static inline -> .bss, no ctor).
    static inline Ring<uint8_t, rx_size> m_rx{};
    static inline Ring<uint8_t, tx_size> m_tx{};

    // Error counters, written in ISRs, read from the main loop. uint8_t
    // reads/writes are atomic on AVR; they wrap at 255. Written as
    // `x = x + 1` because compound ops on volatile are deprecated in C++20.
    static inline volatile uint8_t m_rx_overruns = 0;   // RX ring full, byte lost
    static inline volatile uint8_t m_frame_errors = 0;  // FERR: byte dropped
    static inline volatile uint8_t m_parity_errors = 0; // PERR: byte dropped
    static inline volatile uint8_t m_hw_overruns = 0;   // BUFOVF: bytes lost in HW

    /// The USART register block for this instance (compile-time selection).
    static volatile USART_t &regs() {
        if constexpr (usart_num == 0) return USART0;
        else if constexpr (usart_num == 1) return USART1;
#ifdef USART2
        else if constexpr (usart_num == 2) return USART2;
#endif
#ifdef USART3
        else if constexpr (usart_num == 3) return USART3;
#endif
#ifdef USART4
        else if constexpr (usart_num == 4) return USART4;
#endif
#ifdef USART5
        else if constexpr (usart_num == 5) return USART5;
#endif
        else static_assert(false, "this USART does not exist on this device");
    }

    /// Route this instance through PORTMUX and set the TX/RX pin directions.
    /// Pin positions per the Dx datasheets: default = Px0(TX)/Px1(RX),
    /// alt1 = Px4(TX)/Px5(RX), with x = A,C,F,B,E,G for USART 0..5.
    /// (Family/package differences are the #ifdef guards; this table is the
    /// candidate for a future per-family device header.)
    static void setup_pins() {
        const uint8_t tx_bm = (route == Route::alt1) ? PIN4_bm : PIN0_bm;
        const uint8_t rx_bm = (route == Route::alt1) ? PIN5_bm : PIN1_bm;

        auto config = [&](volatile PORT_t &port) {
            port.OUTSET = tx_bm;           // idle-high BEFORE driving the pin
            port.DIRSET = tx_bm;
            port.DIRCLR = rx_bm;
        };

        if constexpr (usart_num == 0) {
            PORTMUX.USARTROUTEA = (PORTMUX.USARTROUTEA & ~PORTMUX_USART0_gm) |
                (route == Route::alt1 ? PORTMUX_USART0_0_bm : 0);
            config(PORTA);
        } else if constexpr (usart_num == 1) {
            PORTMUX.USARTROUTEA = (PORTMUX.USARTROUTEA & ~PORTMUX_USART1_gm) |
                (route == Route::alt1 ? PORTMUX_USART1_0_bm : 0);
            config(PORTC);
        }
#ifdef USART2
        else if constexpr (usart_num == 2) {
            PORTMUX.USARTROUTEA = (PORTMUX.USARTROUTEA & ~PORTMUX_USART2_gm) |
                (route == Route::alt1 ? PORTMUX_USART2_0_bm : 0);
            config(PORTF);
        }
#endif
#if defined(USART3) && defined(PORTB)
        else if constexpr (usart_num == 3) {
            PORTMUX.USARTROUTEA = (PORTMUX.USARTROUTEA & ~PORTMUX_USART3_gm) |
                (route == Route::alt1 ? PORTMUX_USART3_0_bm : 0);
            config(PORTB);
        }
#endif
#if defined(USART4) && defined(PORTE)
        else if constexpr (usart_num == 4) {
            PORTMUX.USARTROUTEB = (PORTMUX.USARTROUTEB & ~PORTMUX_USART4_gm) |
                (route == Route::alt1 ? PORTMUX_USART4_0_bm : 0);
            config(PORTE);
        }
#endif
#if defined(USART5) && defined(PORTG)
        else if constexpr (usart_num == 5) {
            PORTMUX.USARTROUTEB = (PORTMUX.USARTROUTEB & ~PORTMUX_USART5_gm) |
                (route == Route::alt1 ? PORTMUX_USART5_0_bm : 0);
            config(PORTG);
        }
#endif
        else static_assert(false,
            "this USART (or its port) does not exist on this device");
    }

public:
    /// Instances are empty tags for concept-based call sites (print(serial,...)).
    constexpr Uart() = default;

    // ---- lifecycle ----------------------------------------------------------

    /**
     * @brief Configure pins, PORTMUX, baud rate and enable the USART.
     *
     * Call AFTER the main clock is set up (the baud divisor is computed from
     * F_CPU) and before sei(). The RXC interrupt is enabled here; the DRE
     * interrupt is enabled on demand by write_byte().
     */
    static void init(uint32_t baud) {
        setup_pins();

        regs().BAUD = static_cast<uint16_t>((F_CPU * 4UL + baud / 2UL) / baud);
        regs().CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc |
                       USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;  // 8N1
        regs().CTRLA = USART_RXCIE_bm;
        regs().CTRLB = USART_TXEN_bm | USART_RXEN_bm;

        _delay_ms(10);  // hold TX idle-high so the first start bit is clean
    }

    // ---- ISR bodies ---------------------------------------------------------

    /**
     * @brief RX Complete interrupt body - call from ISR(USARTn_RXC_vect).
     *
     * RXDATAH (status for the byte at the FIFO head) must be read BEFORE
     * RXDATAL (which advances the FIFO). Corrupted bytes (frame/parity) are
     * counted and dropped; BUFOVF means the hardware already lost bytes.
     *
     * @return true when the RX ring transitioned empty -> non-empty: the
     * edge signal for kernel glue ("post RxActivity to the serial AO on
     * true"). Every empty->non-empty transition reports true and the
     * consumer only empties the ring by draining it, so no wakeup is ever
     * lost. Plain (non-kernel) apps may ignore the return value.
     */
    // always_inline: single call site (the ISR binding) - see ticker.hpp
    // pit() for the register-set rationale.
    [[gnu::always_inline]] static bool rxc() {
        const uint8_t status = regs().RXDATAH;
        const uint8_t data = regs().RXDATAL;

        if (status & USART_BUFOVF_bm) {
            m_hw_overruns = m_hw_overruns + 1;
        }
        if (status & (USART_FERR_bm | USART_PERR_bm)) {
            if (status & USART_FERR_bm) m_frame_errors = m_frame_errors + 1;
            if (status & USART_PERR_bm) m_parity_errors = m_parity_errors + 1;
            return false;  // drop the corrupted byte
        }
        const bool was_empty = m_rx.empty_from_isr();
        if (!m_rx.try_put_from_isr(data)) {
            m_rx_overruns = m_rx_overruns + 1;
            return false;  // full ring cannot be empty: no edge
        }
        return was_empty;
    }

    /**
     * @brief Data Register Empty interrupt body - call from ISR(USARTn_DRE_vect).
     *
     * Feeds the next byte from the TX ring; disables itself when the ring
     * drains (write_byte() re-enables it).
     */
    // always_inline: single call site (the ISR binding) - see ticker.hpp
    // pit() for the register-set rationale.
    [[gnu::always_inline]] static void dre() {
        uint8_t c;
        if (m_tx.get_from_isr(c)) {
            regs().TXDATAL = c;
            if (m_tx.empty_from_isr()) {
                regs().CTRLA &= ~USART_DREIE_bm;
            }
        } else {
            regs().CTRLA &= ~USART_DREIE_bm;
        }
    }

    // ---- byte transport (satisfies ByteSink / ByteSource) -------------------

    /// Try to queue one byte for transmission; false when the TX ring is full.
    static bool write_byte(uint8_t b) {
        if (!m_tx.try_put(b)) {
            return false;
        }
        regs().CTRLA |= USART_DREIE_bm;
        return true;
    }

    /// Fetch one received byte; false when nothing is pending.
    static bool read_byte(uint8_t &b) {
        return m_rx.get(b);
    }

    /// Queue as much of the buffer as fits; returns the number queued.
    static uint8_t write(const uint8_t *buffer, uint8_t len) {
        uint8_t written = 0;
        while (written < len && write_byte(buffer[written])) {
            ++written;
        }
        return written;
    }

    // ---- introspection ------------------------------------------------------

    static auto rx_pending() { return m_rx.count(); }
    static bool tx_idle() { return m_tx.empty(); }

    static uint8_t rx_overruns() { return m_rx_overruns; }
    static uint8_t frame_errors() { return m_frame_errors; }
    static uint8_t parity_errors() { return m_parity_errors; }
    static uint8_t hw_overruns() { return m_hw_overruns; }

    static void clear_errors() {
        m_rx_overruns = 0;
        m_frame_errors = 0;
        m_parity_errors = 0;
        m_hw_overruns = 0;
    }
};

static_assert(ByteTransport<Uart<0>>, "Uart must satisfy the transport concepts");

} // namespace brio
