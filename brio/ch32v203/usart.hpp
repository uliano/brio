/*
 * usart.hpp
 *
 * The CH32V203's serial ports (RM ch. 18): the RESOURCE, register by
 * register, and the interrupt-driven byte TRANSPORT written on it.
 *
 * FOUR INSTANCES, TWO BUSES. USART1 sits on PB2 and USART2, USART3 and
 * UART4 on PB1 - which on this family are both HCLK (clock.hpp), so the
 * divisor arithmetic is the same for all four today; the transport asks
 * its own bus all the same, because that is what stays right when a
 * prescaler is unpinned. WHICH instances a part offers is the part's
 * table (device::has_usart), and the list is not always the first n of
 * them: the smallest part offers one usart and it is USART2, because
 * its package bonds neither of USART1's pin pairs.
 *
 * THE PADS ARE PER PART AS WELL AS PER INSTANCE, and UART4 is where that
 * bites: the reference manual has TWO remap tables for it, and the one
 * that starts at PC10/PC11 belongs to the bigger classes - the CH32V203C8
 * is named explicitly in the other (table 10-27), whose default pads are
 * PB0 and PB1. A driver that carried the first table over would drive
 * two pads this package does not bond.
 *
 * WHAT IS HERE TODAY. The frame, the baud generator, the enables, the
 * status flags with the sequences that clear them, the two ISR bodies,
 * and the transport with its two rings - enough for a console and for
 * the suites that run on one.
 *
 * NOT COVERED YET, each with its reason:
 *  - the REMAPS (AFIO_PCFR1/PCFR2): every instance is on its default
 *    pads, which is where this board's console is; they arrive with
 *    afio.hpp and the first program that needs a moved pad.
 *  - mute mode, LIN, IrDA, smartcard, the synchronous mode and the
 *    flow-control pair: the chapter has them all and the CH32V00x
 *    driver spells them out; they are written here when the USART
 *    chapter of this stratum is measured, not before.
 *  - the DMA requests (CTLR3's DMAT/DMAR): the engine slots arrive with
 *    ch32v203/dma.hpp.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/clock.hpp"
#include "ch32v203/device.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "util/clock.hpp"
#include "util/ring.hpp"

namespace brio {

/// DATA bits of a frame. The register speaks in WORD length (M: 8 or 9
/// bits including the parity bit when there is one), so seven data bits
/// exist only with a parity bit and nine only without.
enum class UartBits : uint8_t { seven = 7, eight = 8, nine = 9 };

enum class UartParity : uint8_t { none, even, odd };

/// CTLR2.STOP, all four codes; the half and one-and-a-half stops are the
/// chapter's own (18.10.5).
enum class UartStop : uint8_t { one = 0, half = 1, two = 2, one_and_half = 3 };

/// The frame the two ends agree on. Defaults to 8N1.
struct UartFormat {
    UartBits bits = UartBits::eight;
    UartParity parity = UartParity::none;
    UartStop stop = UartStop::one;
};

constexpr bool uart_format_valid(const UartFormat& f) {
    if (f.bits == UartBits::seven) {
        return f.parity != UartParity::none;
    }
    if (f.bits == UartBits::nine) {
        return f.parity == UartParity::none;
    }
    return true;
}

/// CTLR1's M, PCE and PS for a format (the word length is the data bits
/// plus the parity bit).
constexpr uint16_t usart_ctlr1_format(const UartFormat& f) {
    uint16_t v = 0;
    const bool parity = f.parity != UartParity::none;
    if (f.bits == UartBits::nine || (f.bits == UartBits::eight && parity)) {
        v |= usart_m;
    }
    if (parity) {
        v |= usart_pce;
    }
    if (f.parity == UartParity::odd) {
        v |= usart_ps;
    }
    return v;
}

constexpr uint16_t usart_ctlr2_stop(UartStop s) {
    return static_cast<uint16_t>(static_cast<uint16_t>(s) << usart_stop_shift);
}

/// The mask of the data bits a frame carries in DATAR (the parity bit,
/// when there is one, is the receiver's to check and not the byte's).
constexpr uint16_t uart_data_mask(const UartFormat& f) {
    return f.bits == UartBits::nine ? 0x1FFu : f.bits == UartBits::eight ? 0xFFu : 0x7Fu;
}

/// The divisor this clock and baud ask for: pclk/baud in sixteenths,
/// rounded to nearest so the error is halved. Below 16 there is no whole
/// clock period per sixteenth of a bit and the generator has nothing to
/// divide by - init() refuses such a value.
constexpr uint32_t usart_divisor(uint32_t pclk, uint32_t baud) {
    return baud == 0u ? 0u : (pclk + baud / 2u) / baud;
}

constexpr bool usart_divisor_valid(uint32_t brr) { return brr >= 16u && brr <= 0xFFFFu; }

/// Which bus an instance answers on, and therefore which gate opens it
/// and which clock its divisor counts.
constexpr Bus usart_bus_for(uint8_t n) { return n == 1 ? Bus::pb2 : Bus::pb1; }

constexpr uint32_t usart_gate_for(uint8_t n) {
    return n == 1 ? rcc_pb2_usart1 :
           n == 2 ? rcc_pb1_usart2 :
           n == 3 ? rcc_pb1_usart3 :
           n == 4 ? rcc_pb1_uart4 : 0;
}

constexpr Irq usart_irq_for(uint8_t n) {
    return n == 1 ? Irq::usart1 :
           n == 2 ? Irq::usart2 :
           n == 3 ? Irq::usart3 : Irq::uart4;
}

/// The two pads of an instance in its DEFAULT mapping (RM tables 10-23
/// to 10-27). UART4's row is the CH32V203C8's own table, not the one
/// the bigger classes use - see the file header.
struct UsartPads {
    Pad tx;
    Pad rx;
};

constexpr UsartPads usart_pads_for(uint8_t n) {
    return n == 1 ? UsartPads{{'A', 9}, {'A', 10}} :
           n == 2 ? UsartPads{{'A', 2}, {'A', 3}} :
           n == 3 ? UsartPads{{'B', 10}, {'B', 11}} :
           n == 4 ? UsartPads{{'B', 0}, {'B', 1}} : UsartPads{};
}

// =============================================================================
// Usart<n>: the resource
// =============================================================================

/**
 * One USART instance, register by register. Every verb stores what it
 * names and nothing else. The task below is written on these verbs; a
 * program that wants the chapter beyond the transport reaches them here.
 */
template <uint8_t n>
struct Usart {
    static_assert(usart_base_for(n) != 0, "brio Usart: this family has USART1..3 and UART4");
    static_assert(device::has_usart(n),
                  "brio Usart: this part does not offer that instance (parts/<part>.hpp)");

    Usart() = delete;

    static constexpr uint8_t number = n;
    static constexpr Bus bus = usart_bus_for(n);
    static constexpr UsartPads pads = usart_pads_for(n);

    static UsartRegs& regs() { return *reinterpret_cast<UsartRegs*>(usart_base_for(n)); }

    /// The peripheral's clock gate. Nothing in this file reads a
    /// register before this is on: an unclocked block answers rubbish.
    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(bus, usart_gate_for(n));
        } else {
            Rcc::disable(bus, usart_gate_for(n));
        }
    }

    /// Put the block back to its reset state, gate left as it is.
    static void reset() { Rcc::reset(bus, usart_gate_for(n)); }

    // ---- configuration ----------------------------------------------------

    /// The frame and the divisor, with the port DISABLED: CTLR1's frame
    /// bits and CTLR2's stop bits may only be trusted while UE is clear
    /// or the transmitter idle, and a divisor written under traffic
    /// lands mid-frame.
    static bool configure(const UartFormat& f, uint32_t brr) {
        if (!uart_format_valid(f) || !usart_divisor_valid(brr)) {
            return false;
        }
        regs().CTLR1 = usart_ctlr1_format(f);
        regs().CTLR2 = usart_ctlr2_stop(f.stop);
        regs().BRR = static_cast<uint16_t>(brr);
        return true;
    }

    static void enable(bool on) {
        if (on) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_ue);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~usart_ue);
        }
    }

    static void transmitter(bool on) {
        regs().CTLR1 = static_cast<uint16_t>(on ? (regs().CTLR1 | usart_te)
                                               : (regs().CTLR1 & ~usart_te));
    }

    static void receiver(bool on) {
        regs().CTLR1 = static_cast<uint16_t>(on ? (regs().CTLR1 | usart_re)
                                               : (regs().CTLR1 & ~usart_re));
    }

    /// Arm or disarm one of the interrupt sources of CTLR1.
    static void interrupt(uint16_t enable_bit, bool on) {
        regs().CTLR1 = static_cast<uint16_t>(on ? (regs().CTLR1 | enable_bit)
                                               : (regs().CTLR1 & ~enable_bit));
    }

    // ---- the line ---------------------------------------------------------

    static uint16_t status() { return regs().STATR; }

    /// The word as the receiver has it - nine bits when the frame is
    /// nine bits wide. Reading DATAR is what clears RXNE, and the
    /// STATR-then-DATAR pair is what clears the error flags.
    static uint16_t read_word() { return static_cast<uint16_t>(regs().DATAR); }
    static void write_word(uint16_t w) { regs().DATAR = w; }

    /// Read the errors and clear them, the sequence the chapter
    /// prescribes (18.10.1): the status register, then the data one.
    static uint16_t take_errors() {
        const uint16_t status = regs().STATR;
        const uint16_t errors = static_cast<uint16_t>(status & (usart_pe | usart_fe | usart_ne | usart_ore));
        if (errors != 0u) {
            (void)regs().DATAR;
        }
        return errors;
    }

    /// The baud the port is ACTUALLY running at, from the divisor in the
    /// register - what a program reports instead of what it asked for.
    static uint32_t actual_baud(uint32_t pclk) {
        const uint32_t brr = regs().BRR;
        return brr == 0u ? 0u : pclk / brr;
    }
};

// =============================================================================
// Uart: the transport task
// =============================================================================

/// The peripheral clock an instance counts its divisor in: USART1 is the
/// PB2 one and the other three are PB1's, and the two buses do NOT run
/// at the same rate above the cap ch32v203/clock.hpp states.
template <uint8_t instance, typename C>
constexpr uint32_t usart_bus_hz(C clock) {
    if constexpr (C::is_static) {
        (void)clock;
        return instance == 1u ? C::pclk2_hz : C::pclk1_hz;
    } else {
        (void)clock;
        return instance == 1u ? C::pclk2_hz() : C::pclk1_hz();
    }
}

/// The same answer from an HCLK, which is what a dynamic clock hands a
/// user it is about to rebase: the RCC still holds the prescalers of the
/// rate being left, so the new bus rate is arithmetic and not a read.
template <uint8_t instance>
constexpr uint32_t usart_bus_hz_at(uint32_t hclk) {
    return instance == 1u ? pclk2_hz_at(hclk) : pclk1_hz_at(hclk);
}

/**
 * The interrupt-driven serial port.
 *
 *   using Serial = brio::Uart<1, Platform>;
 *   constexpr Serial serial;                 // tag for print(serial, ...)
 *   Serial::init(clock, 115200);
 *   extern "C" BRIO_CH32_INTERRUPT void usart1_handler() { Serial::isr(); }
 *
 * `P` is the platform, which the rings need to know whether an index can
 * be shared with a handler bare (atomic_width) or wants a guard.
 */
template <uint8_t instance, typename P, uint16_t rx_size = 64, uint16_t tx_size = 64,
          UartFormat format = {}>
struct Uart {
    static_assert(usart_base_for(instance) != 0,
                  "brio Uart: this family has USART1..3 and UART4");
    static_assert(uart_format_valid(format) && format.bits != UartBits::nine,
                  "brio Uart: the rings carry bytes - seven data bits with a parity bit, or eight, "
                  "with or without one; nine-bit words are the resource's read_word()/write_word()");

    Uart() = default;   // a tag instance: constexpr Uart<1, P> serial;

    using Resource = Usart<instance>;

    static constexpr uint8_t number = instance;
    static constexpr UsartPads pads = usart_pads_for(instance);

    using Tx = Pin<pads.tx.port, pads.tx.pin>;
    using Rx = Pin<pads.rx.port, pads.rx.pin>;

    static UsartRegs& regs() { return Resource::regs(); }

    /**
     * Open the port at `baud` in the given frame (8N1 by default), with
     * the receive interrupt armed.
     *
     * Returns false when the divisor comes out below the smallest the
     * chapter allows (16, i.e. one whole peripheral-clock period per bit
     * sixteenth): a baud rate this clock cannot serve is a fact the
     * caller must see rather than a port that mangles every frame.
     */
    template <typename C>
    static bool init(C clock, uint32_t baud) {
        static_assert(clock_follows<C, Uart>(),
                      "brio Uart: the baud divisor is derived from the peripheral clock, so a "
                      "dynamic clock must list this port among the users it rebases");

        const uint32_t pclk = usart_bus_hz<instance>(clock);
        const uint32_t brr = usart_divisor(pclk, baud);
        if (!usart_divisor_valid(brr)) {
            return false;
        }

        Resource::bus_clock(true);

        // Pads before the enable: TE's idle frame must land on a pad the
        // peripheral already owns.
        Tx::function();                  // alternate function, push-pull
        Rx::input();                     // floating: the peer drives it

        if (!Resource::configure(format, brr)) {
            return false;
        }
        m_baud = baud;

        m_tx.clear();
        m_rx.clear();
        m_rx_overruns = 0;
        m_hw_overruns = 0;
        m_frame_errors = 0;
        m_noise_errors = 0;
        m_parity_errors = 0;

        regs().CTLR1 = static_cast<uint16_t>(usart_ctlr1_format(format) | usart_ue | usart_te |
                                             usart_re | usart_rxneie);

        Pfic::enable(usart_irq_for(instance));
        return true;
    }

    /**
     * The port's whole interrupt body - bind this instance's vector to
     * it.
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

    /// Every byte or nothing: spins while the ring is full, which is
    /// what a console wants and what an ISR must never call.
    static void write(const uint8_t* data, uint16_t len) {
        for (uint16_t i = 0; i < len; ++i) {
            while (!write_byte(data[i])) {
            }
        }
    }

    /// Nothing queued and the shift register empty: what a program waits
    /// on before a reset or a clock change.
    static bool tx_idle() {
        return m_tx.empty() && (regs().STATR & usart_tc) != 0u;
    }

    static bool rx_available() { return !m_rx.empty(); }

    // ---- introspection ----------------------------------------------------

    static uint32_t baud() { return m_baud; }
    static uint32_t actual_baud(uint32_t pclk) { return Resource::actual_baud(pclk); }
    static uint16_t rx_overruns() { return m_rx_overruns; }
    static uint16_t hw_overruns() { return m_hw_overruns; }
    static uint16_t frame_errors() { return m_frame_errors; }
    static uint16_t noise_errors() { return m_noise_errors; }
    static uint16_t parity_errors() { return m_parity_errors; }

    /// Follow a clock that changed rate. The argument is HCLK - what a
    /// dynamic clock hands every user - and this port derives its own
    /// bus rate from it, because the two peripheral buses do not divide
    /// the same number.
    ///
    /// It is called BEFORE the rate moves, with the old clock still
    /// running, so the queue is drained here: a byte still in the shift
    /// register when SYSCLK changes goes out at neither baud rate. The
    /// drain is bounded - a port whose line is held off must not hang
    /// the switch - and the divisor is written whatever the drain found.
    static void rebase(uint32_t hz) {
        constexpr uint32_t drain_spins = 2'000'000UL;
        uint32_t spins = drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = drain_spins;
        while ((regs().STATR & usart_tc) == 0u && spins-- != 0u) {
        }
        const uint32_t brr = usart_divisor(usart_bus_hz_at<instance>(hz), m_baud);
        if (usart_divisor_valid(brr)) {
            regs().BRR = static_cast<uint16_t>(brr);
        }
    }

private:
    /// Saturating: a counter that wraps would report a healthy port.
    static void bump(uint16_t& counter) {
        if (counter != 0xFFFFu) {
            ++counter;
        }
    }

    static inline Ring<uint8_t, rx_size, P> m_rx{};
    static inline Ring<uint8_t, tx_size, P> m_tx{};
    static inline uint32_t m_baud = 0;
    static inline uint16_t m_rx_overruns = 0;
    static inline uint16_t m_hw_overruns = 0;
    static inline uint16_t m_frame_errors = 0;
    static inline uint16_t m_noise_errors = 0;
    static inline uint16_t m_parity_errors = 0;
};

} // namespace brio
