/*
 * usart.hpp
 *
 * The USART (RM0444 ch. 33) in the two strata every brio serial driver
 * has (docs/design/serial.md):
 *
 *  Usart<n>              the RESOURCE: which instance, where its registers
 *                        are, its bus clock, its kernel-clock multiplexer
 *                        where it has one, its NVIC line, the enable
 *                        discipline and the flag surface;
 *  Uart<n, pins, rx, tx> the TASK: the asynchronous 8N1-class byte
 *                        transport with ring buffers and one ISR body -
 *                        every console's personality, a ByteTransport
 *                        for print()/SerialPort. The SAME public surface
 *                        as avrdx's Uart and samc's Uart, which is what
 *                        lets util/serial_port.hpp and print() compile
 *                        on the third architecture untouched.
 *
 * SCOPE, honestly: the asynchronous, oversampling-by-16, PCLK-clocked
 * USART with 7/8/9-bit words, parity and 1/2 stop bits, interrupt-driven
 * in the non-FIFO register view. The chapter's long tail is DECLARED and
 * not built: the FIFO mode (USART1..3 on the G0B1 have 8-deep FIFOs, the
 * others do not - RM0444 table 183 - and which instance has one is a
 * pointer-comparison macro in the device header, not a constexpr fact),
 * the PRESC prescaler (same instance split), oversampling by 8, the
 * kernel-clock choices other than PCLK (HSI16 and LSE, which are what
 * lets an instance run and wake in Stop), synchronous mode, hardware
 * flow control, single-wire, LIN, IrDA, smartcard, Modbus, auto-baud,
 * the receiver time-out, DMA, the LPUARTs. Each arrives with the pass
 * that measures it (docs/stm32g0/usart.md, "Not covered yet").
 *
 * Facts that shape the code (RM0444 33.5, 33.8; ES0548 rev Z):
 *  - BRR, CR1's frame fields, CR2, CR3 and PRESC are written only with
 *    UE = 0 (33.8.x "can only be written when the USART is disabled");
 *    the task disables around every configuration, including rebase();
 *  - the baud generator: BRR = USARTDIV = usart_ker_ck / baud with
 *    OVER8 = 0, USARTDIV >= 16 (33.5.7) - a plain integer divisor, the
 *    simplest of the three targets (no 65536-scaled fraction, no
 *    fractional fbaud), rounded to nearest and reported back through
 *    actual_baud();
 *  - TXE is a CONDITION (transmit data register empty), so its interrupt
 *    is armed only while the ring holds something and disarmed from the
 *    handler when it runs dry - the samc DRE discipline;
 *  - ORE raises the interrupt whenever RXNEIE is set (33.8.9), and it is
 *    cleared ONLY through ICR.ORECF - a handler that reads RDR and
 *    leaves ORE standing re-enters for ever (the SERCOM ERROR storm the
 *    samc bench caught, in this family's clothes). Every error flag has
 *    its ICR twin and the handler clears what it counts;
 *  - RDR holds the LAST GOOD byte when ORE is set (the lost one is the
 *    next); FE/NE/PE flags in the non-FIFO view belong to the byte in
 *    RDR, so a framed or parity-failed byte is dropped precisely;
 *  - TE set sends an idle frame first (33.5.5), which is why the pads
 *    are handed to the peripheral BEFORE UE/TE are raised;
 *  - ES0548 2.11.1: a sub-half-bit glitch to zero inside the second half
 *    of the stop bit corrupts the received byte, no workaround - the
 *    noise flag NE is what such a line shows first, and the task counts
 *    it separately for that reason;
 *  - the vector is SHARED on this family (device_tables.hpp): the app's
 *    handler for USART2_LPUART2_IRQHandler calls this instance's isr()
 *    and, if it uses LPUART2, that one's too.
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <span>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "stm32g0/platform_stm32.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"

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

// ---- baud arithmetic (33.5.7, OVER8 = 0) ---------------------------------------

/// BRR for `baud` at kernel clock `hz`: USARTDIV = hz / baud, rounded to
/// nearest, legal in 16..65535. Nothing when the rate is unreachable.
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

/// What the generator really produces for a BRR value.
constexpr uint32_t usart_actual_baud(uint32_t hz, uint16_t brr) {
    return brr == 0u ? 0u : hz / brr;
}

/// The lowest kernel clock that still reaches `baud` (USARTDIV >= 16).
constexpr uint32_t usart_min_hz(uint32_t baud) { return baud * 16u; }

// ---- the resource -------------------------------------------------------------

/// USART_ISR bits a byte transport looks at, by their non-FIFO names.
struct UsartFlag {
    static constexpr uint32_t rxne = USART_ISR_RXNE_RXFNE;
    static constexpr uint32_t txe = USART_ISR_TXE_TXFNF;
    static constexpr uint32_t tc = USART_ISR_TC;
    static constexpr uint32_t ore = USART_ISR_ORE;
    static constexpr uint32_t fe = USART_ISR_FE;
    static constexpr uint32_t ne = USART_ISR_NE;
    static constexpr uint32_t pe = USART_ISR_PE;
    static constexpr uint32_t receive_errors = ore | fe | ne | pe;
};

/// The kernel clock an instance with a multiplexer may take (CCIPR
/// USARTnSEL codes). Only `pclk` is driven by this stratum.
enum class UsartClock : uint8_t { pclk = 0, sysclk = 1, hsi16 = 2, lse = 3 };

/**
 * Usart<n>: the instance. Every verb is one register fact; the ORDER
 * (configure disabled, pads before enable) is the task's.
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

    static USART_TypeDef& regs() { return *reinterpret_cast<USART_TypeDef*>(usart_base(n)); }
    static constexpr IRQn_Type irq() { return usart_irq(n); }

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

    static uint16_t brr() { return static_cast<uint16_t>(regs().BRR); }

    static uint32_t status() { return regs().ISR; }
    static void clear_flags(uint32_t icr_mask) { regs().ICR = icr_mask; }
    static uint8_t read_data() { return static_cast<uint8_t>(regs().RDR); }
    static void write_data(uint8_t b) { regs().TDR = b; }

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

/**
 * Uart<n, pins, rx_size, tx_size>: the interrupt-driven byte transport.
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
 * Same verbs, same return contracts as the other two targets' Uart:
 * init/rebase/isr/write_byte/read_byte/write/write_bulk/read_bulk, the
 * error counters, release(). The DMA engine slots the samc Uart carries
 * are absent here until the DMA campaign - the template shape leaves
 * the room.
 */
template <uint8_t n, UartPins pins, uint32_t rx_size = 64, uint32_t tx_size = 256>
class Uart {
    using S = Usart<n>;

    static_assert(uart_pins_valid(pins),
                  "brio Uart: the two pads must be real pins of present ports, and TX "
                  "and RX cannot be the same pad");

    using TxPin = Pin<pins.tx.port, pins.tx.pin>;
    using RxPin = Pin<pins.rx.port, pins.rx.pin>;

    static inline Ring<uint8_t, rx_size, Stm32Platform> m_rx{};
    static inline Ring<uint8_t, tx_size, Stm32Platform> m_tx{};

    static inline volatile uint8_t m_rx_overruns = 0;   // RX ring full, byte lost
    static inline volatile uint8_t m_frame_errors = 0;  // FE: byte dropped
    static inline volatile uint8_t m_parity_errors = 0; // PE: byte dropped
    static inline volatile uint8_t m_noise_errors = 0;  // NE: byte kept, line suspect
    static inline volatile uint8_t m_hw_overruns = 0;   // ORE: a byte lost in silicon
    static inline uint32_t m_baud = 0;                  // for rebase()

public:
    constexpr Uart() = default;

    using Resource = S;

    /**
     * @brief Bring the instance up: bus clock, kernel clock, frame, baud,
     * pads, the receive interrupt and its NVIC line.
     *
     * Call AFTER the main clock is set up and before interrupts are
     * enabled globally; `clock` is the app's brio::Clock tag, and the
     * divisor comes from its PCLK rate - never from a second statement
     * of the rate. False when the rate cannot be produced at this clock:
     * the caller then knows the transport is NOT up, rather than
     * printing into a ring nothing will drain.
     */
    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, const UartFormat& format = {}) {
        static_assert(clock_follows<Clock, Uart>(),
                      "this Uart is initialized with a DynamicClock that does not "
                      "list it among its Users: it would keep the old baud after "
                      "a clock change");
        (void)clock;

        const std::optional<uint16_t> reg = usart_brr(Clock::pclk_hz, baud);
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
        if (!S::kernel_clock(UsartClock::pclk)) {
            return false;
        }
        if (!S::configure(format, *reg)) {
            return false;
        }

        // Pads BEFORE the enable: TE's idle frame must land on a pad the
        // peripheral already owns. The RX pad gets a pull-up so an
        // unconnected line reads idle rather than noise.
        TxPin::function(pins.tx.function);
        RxPin::function(pins.rx.function, {.pull = PinPull::up});

        S::enable(true);
        S::clear_flags(USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF);
        S::rxne_interrupt(true);
        Nvic::enable(S::irq());
        return true;
    }

    /**
     * @brief The core clock changed (DynamicClock fan-out): keep the
     * same bit rate at the new rate. Called BEFORE the clock changes,
     * so the drain runs at the rate the queued bytes were meant for;
     * TC answers "the shifter is empty" exactly. BRR is UE-protected,
     * so the instance is stopped around the write. Main context only.
     */
    static void rebase(uint32_t hz) {
        constexpr uint32_t ring_drain_spins = 8'000'000u;
        constexpr uint32_t frame_spins = 200'000u;
        uint32_t spins = ring_drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = frame_spins;
        while ((S::status() & UsartFlag::tc) == 0u && spins-- != 0u) {
        }
        const std::optional<uint16_t> reg = usart_brr(hz, m_baud);
        if (!reg) {
            return;
        }
        S::enable(false);
        S::regs().BRR = *reg;
        S::enable(true);
    }

    static constexpr uint32_t min_hz_for(uint32_t baud) { return usart_min_hz(baud); }
    static constexpr bool can_baud(uint32_t hz, uint32_t baud) {
        return usart_brr(hz, baud).has_value();
    }
    static uint32_t actual_baud(uint32_t hz) { return usart_actual_baud(hz, S::brr()); }

    /**
     * @brief The instance's ONE interrupt body - call from the vector
     * the device gives this instance (shared with others on this family).
     *
     * @return true when the RX ring transitioned empty -> non-empty: the
     * edge signal for kernel glue ("post RxActivity on true"). Every
     * empty->non-empty transition reports true and the consumer only
     * empties the ring by draining it, so no wakeup is ever lost.
     */
    [[gnu::always_inline]] static bool isr() {
        USART_TypeDef& r = S::regs();
        const uint32_t st = r.ISR;
        bool edge = false;

        if ((st & UsartFlag::rxne) != 0u) {
            const uint8_t b = static_cast<uint8_t>(r.RDR);   // clears RXNE
            const uint32_t err = st & (UsartFlag::fe | UsartFlag::ne | UsartFlag::pe);
            if (err != 0u) {
                r.ICR = err;   // the ICR bits sit at the ISR positions
                if ((err & UsartFlag::fe) != 0u) {
                    m_frame_errors = m_frame_errors + 1;
                }
                if ((err & UsartFlag::pe) != 0u) {
                    m_parity_errors = m_parity_errors + 1;
                }
                if ((err & UsartFlag::ne) != 0u) {
                    m_noise_errors = m_noise_errors + 1;
                }
            }
            if ((err & (UsartFlag::fe | UsartFlag::pe)) == 0u) {
                const bool was_empty = m_rx.empty();
                if (m_rx.push(b)) {
                    edge = was_empty;
                } else {
                    m_rx_overruns = m_rx_overruns + 1;
                }
            }
        }
        if ((st & UsartFlag::ore) != 0u) {
            r.ICR = USART_ICR_ORECF;   // or this handler re-enters for ever
            m_hw_overruns = m_hw_overruns + 1;
        }

        if ((st & UsartFlag::txe) != 0u && (r.CR1 & USART_CR1_TXEIE_TXFNFIE) != 0u) {
            const auto v = m_tx.pop();
            if (v) {
                r.TDR = *v;
            } else {
                r.CR1 = r.CR1 & ~USART_CR1_TXEIE_TXFNFIE;   // ring dry: TXE would re-fire
            }
        }
        return edge;
    }

    /// Queue one byte; false when the ring is full (print() retries).
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            return false;
        }
        S::txe_interrupt(true);
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

    /// Queue a run of bytes through the ring's contiguous span and nudge
    /// the transmitter ONCE - the bulk verb the samc campaign measured
    /// the per-byte one against. Returns the number queued.
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
        if (queued != 0u) {
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
    }

    /// Put everything back: interrupts off, instance disabled and reset,
    /// pads to analog, bus clock off. The rings are dropped.
    static void release() {
        Nvic::disable(S::irq());
        S::enable(false);
        S::reset();
        TxPin::release();
        RxPin::release();
        S::bus_clock(false);
        m_rx.clear();
        m_tx.clear();
    }
};

} // namespace brio
