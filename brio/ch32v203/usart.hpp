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
 * TWO OPTIONAL DMA ENGINE SLOTS, the shape every stratum with a
 * controller uses: a transmit engine drains the TX ring by contiguous
 * runs (the ring's read_span, consumed by exactly what the block
 * carried) and a receive engine fills the RX ring's free run
 * (write_span, published by what harvest() finds arrived). Without an
 * engine the slot is the NoDmaEngine tag, every engine branch is
 * compiled out and init() does not so much as touch CTLR3. With a
 * receive engine RXNE belongs to the channel, so the error flags are
 * read once per harvest and counted against the RUN and not the byte -
 * a console that wants exact attribution takes no receive engine. And
 * harvest() is a VERB, not an interrupt: a receive block completes only
 * when its run fills, which on an idle line is never, so whoever owns
 * the port decides how often to ask. An engine is refused on any
 * channel but the instance's own, which the request table of
 * ch32v203/dma_engine.hpp answers (USART1 transmits on channel 4 and
 * receives on 5, USART2 on 7 and 6, USART3 on 2 and 3, UART4 on 1
 * and 8) - and a program with an engine running does not sleep on this
 * family, because in Sleep the bus matrix serves the core alone
 * (docs/ch32v203/dma.md).
 *
 * NOT COVERED YET, each with its reason:
 *  - the REMAPS (AFIO_PCFR1/PCFR2): every instance is on its default
 *    pads, which is where this board's console is; they arrive with
 *    afio.hpp and the first program that needs a moved pad.
 *  - mute mode, LIN, IrDA, smartcard, the synchronous mode and the
 *    flow-control pair: the chapter has them all and the CH32V00x
 *    driver spells them out; they are written here when the USART
 *    chapter of this stratum is measured, not before.
 */

#pragma once

#include <stdint.h>

#include "ch32v203/clock.hpp"
#include "ch32v203/device.hpp"
#include "ch32v203/dma_engine.hpp"
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

/// The DMA channel each direction of an instance answers on, read out
/// of the one request table (ch32v203/dma_engine.hpp): 0 where the part
/// has not got the instance at all. THE CHANNEL IS THE REQUEST here, so
/// these two numbers are what an engine slot is checked against.
constexpr uint8_t usart_dma_tx_channel(uint8_t n) {
    return n == 1 ? dma_request_channel(DmaRequest::usart1_tx) :
           n == 2 ? dma_request_channel(DmaRequest::usart2_tx) :
           n == 3 ? dma_request_channel(DmaRequest::usart3_tx) :
           n == 4 ? dma_request_channel(DmaRequest::uart4_tx) : 0;
}

constexpr uint8_t usart_dma_rx_channel(uint8_t n) {
    return n == 1 ? dma_request_channel(DmaRequest::usart1_rx) :
           n == 2 ? dma_request_channel(DmaRequest::usart2_rx) :
           n == 3 ? dma_request_channel(DmaRequest::usart3_rx) :
           n == 4 ? dma_request_channel(DmaRequest::uart4_rx) : 0;
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

    /// CTLR3's two DMA request enables (18.10.6). With DMAT set, TXE
    /// raises a request instead of feeding the interrupt; with DMAR
    /// set, so does RXNE - which is why a receiver with an engine
    /// leaves RXNEIE alone.
    static void dma_transmit(bool on) {
        regs().CTLR3 = static_cast<uint16_t>(on ? (regs().CTLR3 | usart_dmat)
                                               : (regs().CTLR3 & ~usart_dmat));
    }
    static void dma_receive(bool on) {
        regs().CTLR3 = static_cast<uint16_t>(on ? (regs().CTLR3 | usart_dmar)
                                               : (regs().CTLR3 & ~usart_dmar));
    }

    /// Where an engine points: the one register both directions share.
    static volatile void* data_address() { return static_cast<volatile void*>(&regs().DATAR); }

    static constexpr uint8_t dma_tx_channel = usart_dma_tx_channel(n);
    static constexpr uint8_t dma_rx_channel = usart_dma_rx_channel(n);

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
          UartFormat format = {}, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
struct Uart {
    static_assert(usart_base_for(instance) != 0,
                  "brio Uart: this family has USART1..3 and UART4");
    static_assert(uart_format_valid(format) && format.bits != UartBits::nine,
                  "brio Uart: the rings carry bytes - seven data bits with a parity bit, or eight, "
                  "with or without one; nine-bit words are the resource's read_word()/write_word()");
    // An engine is checked where it is NAMED: sizeof demands a complete
    // type, so an engine's own static_asserts fire on the line the
    // application wrote.
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / DmaRxEngine from "
                  "ch32v203/dma.hpp, or NoDmaEngine (the default)");
    static_assert(!TxEngine::present ||
                      dma_engine_channel<TxEngine>() == usart_dma_tx_channel(instance),
                  "brio Uart: table 11-5 - this instance transmits on its own DMA channel "
                  "(USART1 on 4, USART2 on 7, USART3 on 2, UART4 on 1)");
    static_assert(!RxEngine::present ||
                      dma_engine_channel<RxEngine>() == usart_dma_rx_channel(instance),
                  "brio Uart: table 11-5 - this instance receives on its own DMA channel "
                  "(USART1 on 5, USART2 on 6, USART3 on 3, UART4 on 8)");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "the two engines of a Uart must not share a DMA channel");

    Uart() = default;   // a tag instance: constexpr Uart<1, P> serial;

    using Resource = Usart<instance>;

    static constexpr uint8_t number = instance;
    static constexpr UsartPads pads = usart_pads_for(instance);
    static constexpr bool has_tx_engine = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;

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

        // CTLR3 IS TOUCHED ONLY BY A PORT THAT HAS AN ENGINE: without
        // one there is nothing of this transport's to put in it, and
        // whatever a program stored there itself stays.
        if constexpr (has_tx_engine || has_rx_engine) {
            m_dma_faults = 0;
            uint16_t ctlr3 = regs().CTLR3;
            if constexpr (has_tx_engine) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_dmat); }
            if constexpr (has_rx_engine) { ctlr3 = static_cast<uint16_t>(ctlr3 | usart_dmar); }
            regs().CTLR3 = ctlr3;
        }

        // RXNE belongs to the receive channel when an engine has it, so
        // the per-byte interrupt is armed only where no engine is.
        regs().CTLR1 = static_cast<uint16_t>(usart_ctlr1_format(format) | usart_ue | usart_te |
                                             usart_re |
                                             (has_rx_engine ? uint16_t{0} : usart_rxneie));

        if constexpr (has_rx_engine) {
            RxEngine::arm(Resource::data_address());
            rearm_rx();
        }
        if constexpr (has_tx_engine) {
            TxEngine::arm(Resource::data_address());
        }

        Pfic::enable(usart_irq_for(instance));
        return true;
    }

    /**
     * The ISR body of WHICHEVER DMA channel this transport owns - bind
     * it to the vector(s) the engines' channels report on:
     *
     *     extern "C" BRIO_CH32_INTERRUPT void dma1_channel7_handler() { (void)Serial::dma_isr(); }
     *
     * Each engine reads only its own channel's flags, so this is safe
     * on a vector another channel of the program shares nothing with.
     * On the transmit channel a completion releases exactly the block's
     * bytes from the ring and starts the next run; on the receive
     * channel nothing is published here - harvest() does that.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        bool mine = false;
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service();
            if ((f & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                bump(m_dma_faults);
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
                bump(m_dma_faults);
                mine = true;
            } else if ((f & RxEngine::flag_complete) != 0u) {
                mine = true;   // the run filled: harvest() publishes and re-arms
            }
        }
        return mine;
    }

    /**
     * Ask the receive engine what has arrived, and publish it. A VERB,
     * not an interrupt (see the file header): the owner decides how
     * often, a kernel TimeEvent every few ticks is the shape. Returns
     * the same edge isr() does - the ring went from empty to non-empty.
     * False, and free, without an engine.
     */
    static bool harvest() {
        if constexpr (!has_rx_engine) {
            return false;
        } else {
            // The error flags once, at harvest granularity, cleared by
            // the STATR-then-DATAR read the chapter prescribes.
            const uint16_t status = regs().STATR;
            if ((status & (usart_fe | usart_ne | usart_pe | usart_ore)) != 0u) {
                if ((status & usart_fe) != 0u) { bump(m_frame_errors); }
                if ((status & usart_ne) != 0u) { bump(m_noise_errors); }
                if ((status & usart_pe) != 0u) { bump(m_parity_errors); }
                if ((status & usart_ore) != 0u) { bump(m_hw_overruns); }
                (void)regs().DATAR;
            }
            const bool was_empty = m_rx.empty();
            const uint16_t fresh = RxEngine::take();
            if (fresh != 0u) {
                m_rx.publish(static_cast<typename decltype(m_rx)::index_t>(fresh));
            }
            // The silicon is asked first and the arithmetic second: a
            // channel that is not running gets a new run whatever the
            // count says.
            if (RxEngine::idle() || RxEngine::full() || RxEngine::capacity() == 0u) {
                rearm_rx();
            }
            return was_empty && !m_rx.empty();
        }
    }

    /// Blocks the engines threw away. Always 0, and free, without one.
    static uint16_t dma_faults() { return m_dma_faults; }

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
        if constexpr (has_tx_engine) {
            pump_tx();
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | usart_txeie);
        }
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
        if constexpr (has_tx_engine) {
            if (TxEngine::busy()) {
                return false;
            }
        }
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
            if constexpr (has_tx_engine) {
                pump_tx();
            }
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
    /// Start the next contiguous run of the TX ring on the engine, if it
    /// is idle and there is one. Under the guard: the completion path
    /// runs in the channel's handler.
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            typename P::CriticalSection cs;
            if (TxEngine::busy()) {
                return;
            }
            const auto run = m_tx.read_span();
            if (run.empty()) {
                return;
            }
            (void)TxEngine::start(run.data(), static_cast<uint16_t>(run.size()));
        }
    }

    /// Point the receive engine at the ring's next free run. No room is
    /// a byte lost before it arrives, counted as the software overrun
    /// it is.
    static void rearm_rx() {
        if constexpr (has_rx_engine) {
            const auto room = m_rx.write_span();
            if (room.empty()) {
                bump(m_rx_overruns);
                return;
            }
            (void)RxEngine::start(room.data(), static_cast<uint16_t>(room.size()));
        }
    }

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
    static inline uint16_t m_dma_faults = 0;
};

} // namespace brio
