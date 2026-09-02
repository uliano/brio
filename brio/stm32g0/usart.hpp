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
 * the receiver time-out, the LPUARTs. Each arrives with the pass that
 * measures it (docs/stm32g0/usart.md, "Not covered yet"). DMA is no
 * longer among them: the task carries two OPTIONAL engine slots, empty
 * by default and costing an engineless image nothing (stm32g0/dma.hpp,
 * NoDmaEngine).
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

    /// Where a DMA channel reads from and writes to. Two REGISTERS here,
    /// where the SERCOM had one DATA: a receive engine points at RDR and
    /// a transmit engine at TDR, and neither can be mistaken for the
    /// other.
    static volatile void* tx_data_address() { return &regs().TDR; }
    static volatile void* rx_data_address() { return &regs().RDR; }

    /**
     * CR3.DMAT / CR3.DMAR (33.8.5): whether the peripheral raises a DMA
     * REQUEST for the condition, instead of - or as well as - the
     * interrupt. With the bit set, TXE (or RXNE) drives the request line
     * and the channel clears the condition by writing (or reading) the
     * data register, so the matching interrupt must NOT also be armed:
     * both would serve the same byte.
     *
     * Not part of configure(): CR3 is UE-protected as a whole, but these
     * two bits are what an engine turns on AFTER the frame is settled,
     * and an application without an engine never touches them.
     */
    static bool dma_transmit(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR3 = on ? (regs().CR3 | USART_CR3_DMAT) : (regs().CR3 & ~USART_CR3_DMAT);
        return true;
    }
    static bool dma_receive(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR3 = on ? (regs().CR3 | USART_CR3_DMAR) : (regs().CR3 & ~USART_CR3_DMAR);
        return true;
    }

    /**
     * THE DMAMUX REQUEST LINES THIS INSTANCE PUBLISHES (RM0444 table 55).
     *
     * They live here and not in stm32g0/device_tables.hpp because NO
     * DEVICE HEADER OF THIS PACK DECLARES THEM - the DMAMUX_REQ_*
     * spellings are ST's HAL/LL, which this project does not vendor - and
     * because of the standing ruling the samc EVSYS campaign settled: a
     * fabric driver owns the fabric, a peripheral owns its own
     * vocabulary. stm32g0/dma.hpp therefore takes a plain request id and
     * knows nothing about USARTs.
     *
     * The numbers are the same on every part of the family; an instance
     * a part does not bond simply has no user for its row.
     */
    static constexpr uint8_t dma_rx_request() {
        switch (n) {
            case 1: return 50;
            case 2: return 52;
            case 3: return 54;
            case 4: return 56;
            case 5: return 74;
            default: return 76;
        }
    }
    static constexpr uint8_t dma_tx_request() {
        switch (n) {
            case 1: return 51;
            case 2: return 53;
            case 3: return 55;
            case 4: return 57;
            case 5: return 75;
            default: return 77;
        }
    }

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
 * The "no DMA engine" default of the two optional Uart engine slots.
 *
 * It is a TAG, not a base class: `present` is the only thing the task
 * asks about, and it asks with `if constexpr`, so every engine branch -
 * the pump, the harvest, the completion path and the state they need -
 * disappears from a Uart that does not name one. The proof is the
 * measured kind: the release images of every app that uses no engine are
 * BYTE-IDENTICAL to the ones built before these parameters existed
 * (docs/stm32g0/dma.md records the gate).
 *
 * IT LIVES HERE AND NOT IN stm32g0/dma.hpp, and the reason is the whole
 * point of an optional slot: usart.hpp must not include dma.hpp, or every
 * program with a console would carry the DMA driver. An application that
 * wants an engine includes both headers and names the channel; one that
 * does not never sees the controller at all - which is also why the task
 * below reaches its engines only through THEIR OWN published names
 * (service(), flag_complete, flag_error) and never spells a DmaChannel or
 * a DmaFlag.
 */
struct NoDmaEngine {
    NoDmaEngine() = delete;
    static constexpr bool present = false;
};

/// Two engines on one transport must not name the same DMA channel: a
/// channel moves data ONE way, and pointing both directions at it would
/// have each re-programming the other's block. Generic over any engine
/// that says `present`, `controller` and `channel` - `if constexpr` keeps
/// those from being looked up on an absent engine, which is what lets
/// NoDmaEngine stay a two-line tag.
template <typename Tx, typename Rx>
constexpr bool uart_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::controller != Rx::controller || Tx::channel != Rx::channel;
    } else {
        return true;
    }
}

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
 * error counters, release().
 *
 * THE TWO OPTIONAL ENGINE SLOTS are the samc Uart's, in this family's
 * clothes: name a stm32g0/dma.hpp DmaTxEngine and/or DmaRxEngine and the
 * bytes move without the CPU; name neither (the default) and every engine
 * branch below disappears - `if constexpr` throughout, and the engineless
 * release images are BYTE-IDENTICAL to the ones built before the
 * parameters existed (docs/stm32g0/dma.md records the md5 gate).
 *
 * WHICHEVER DIRECTION HAS AN ENGINE DOES NOT ARM ITS INTERRUPT: the DMA
 * request and the interrupt are the SAME condition (TXE, RXNE), so arming
 * both would have the channel and the handler each serve one byte.
 *
 * WHAT AN RX ENGINE TRADES AWAY, and it cannot be given back: per-byte
 * error attribution. With RXNEIE armed, ISR is read for EACH character
 * before its RDR and a framed or parity-failed byte is dropped precisely.
 * With a channel consuming RXNE instead, nobody reads ISR per character -
 * harvest() reads it once per harvest and counts what it finds, which is
 * the honest resolution of that mode.
 */
template <uint8_t n, UartPins pins, uint32_t rx_size = 64, uint32_t tx_size = 256,
          typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
class Uart {
    using S = Usart<n>;

    static_assert(uart_pins_valid(pins),
                  "brio Uart: the two pads must be real pins of present ports, and TX "
                  "and RX cannot be the same pad");

    // AN ENGINE IS CHECKED WHERE IT IS NAMED. `sizeof` demands a COMPLETE
    // type, which instantiates the engine right here, at the template
    // argument the application typed - so an engine's own static_asserts
    // (its channel number above all) fire on that line. Without this the
    // engine stays uninstantiated until something first touches it, and a
    // Uart carrying an impossible channel compiles perfectly happily.
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from stm32g0/dma.hpp, or NoDmaEngine (the default)");
    static_assert(uart_engines_distinct<TxEngine, RxEngine>(),
                  "the transmit and receive engines must use DIFFERENT DMA "
                  "channels: a channel moves data in one direction only");

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

    /// DMA blocks this transport had to throw away. Touched only from
    /// inside `if constexpr (has_*_engine)` branches, so an engineless
    /// build never reads or writes it.
    static inline volatile uint8_t m_dma_faults = 0;

public:
    constexpr Uart() = default;

    using Resource = S;

    /// Whether this instantiation carries an engine at all. Every engine
    /// branch below is `if constexpr` on these, so a false one costs
    /// nothing - not a byte, not a branch.
    static constexpr bool has_tx_engine = TxEngine::present;
    static constexpr bool has_rx_engine = RxEngine::present;

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

        // CR3's two request bits are UE-protected like the rest of the
        // register, so they go in here, before the enable.
        if constexpr (has_tx_engine) {
            (void)S::dma_transmit(true);
        }
        if constexpr (has_rx_engine) {
            (void)S::dma_receive(true);
        }

        S::enable(true);
        S::clear_flags(USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF);

        if constexpr (has_rx_engine) {
            // NOT S::rxne_interrupt(true): the channel consumes RXNE.
            RxEngine::arm(S::rx_data_address(), S::dma_rx_request());
            rearm_rx();
        } else {
            S::rxne_interrupt(true);
        }
        if constexpr (has_tx_engine) {
            TxEngine::arm(S::tx_data_address(), S::dma_tx_request());
        }
        // TXE is armed on demand by write_byte() when there is no engine.

        Nvic::enable(S::irq());
        return true;
    }

    /**
     * @brief The ISR body of WHICHEVER DMA channel this transport owns -
     * call it from the vector(s) the engines' channels report on.
     *
     *     extern "C" void DMA1_Channel1_IRQHandler() { (void)Serial::dma_isr(); }
     *
     * Each engine's channel reads ONLY its own flags (there is no
     * "which channel interrupted" register on this controller), so this is
     * safe to call from a shared vector that also serves other people's
     * channels: it answers for its own and returns false otherwise.
     *
     * On the transmit channel a completion means the block has left the
     * ring, so exactly that many bytes are released and the next
     * contiguous run started. On the receive channel nothing is published
     * here - only harvest() knows how much of the run the consumer has
     * been told about, and the pacing of that is the owner's.
     *
     * @return true when something belonging to this transport was served.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        bool mine = false;
        if constexpr (has_tx_engine) {
            const uint8_t f = TxEngine::service();
            if ((f & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                m_dma_faults = m_dma_faults + 1;
                mine = true;
            } else if ((f & TxEngine::flag_complete) != 0u) {
                m_tx.consume(static_cast<typename decltype(m_tx)::index_t>(
                    TxEngine::complete()));
                pump_tx();
                mine = true;
            }
        }
        if constexpr (has_rx_engine) {
            const uint8_t f = RxEngine::service();
            if ((f & RxEngine::flag_error) != 0u) {
                (void)RxEngine::abandon();
                m_dma_faults = m_dma_faults + 1;
                mine = true;
            } else if ((f & RxEngine::flag_complete) != 0u) {
                // The run filled up. harvest() publishes and re-arms.
                mine = true;
            }
        }
        return mine;
    }

    /**
     * @brief Ask the receive engine what has arrived, and publish it.
     *
     * WHY THIS IS A VERB AND NOT AN INTERRUPT. A receive block completes
     * only when the buffer fills, which on an idle line may be never, so
     * there is no event to wait for. WHOEVER OWNS THE PORT DECIDES HOW
     * OFTEN TO ASK and pays the latency it chose; a kernel TimeEvent every
     * few ticks is the shape brio expects.
     *
     * On this silicon the asking itself is nearly free - one CNDTR read
     * (stm32g0/dma.hpp, DmaRxEngine::take()) where the SAM had to suspend
     * the channel and validate a write-back against an erratum.
     *
     * @return true when the receive ring went from empty to non-empty -
     * the same edge contract isr() has, so the same kernel glue works.
     * False, and free, without an engine.
     */
    static bool harvest() {
        if constexpr (!has_rx_engine) {
            return false;
        } else {
            // The error flags once, at harvest granularity: nobody read
            // ISR per character, so this is the resolution this mode has.
            const uint32_t st = S::status();
            const uint32_t errors = st & UsartFlag::receive_errors;
            if (errors != 0u) {
                S::clear_flags(errors);   // the ICR bits sit at the ISR positions
                if ((errors & UsartFlag::ore) != 0u) {
                    m_hw_overruns = m_hw_overruns + 1;
                }
                if ((errors & UsartFlag::fe) != 0u) {
                    m_frame_errors = m_frame_errors + 1;
                }
                if ((errors & UsartFlag::pe) != 0u) {
                    m_parity_errors = m_parity_errors + 1;
                }
                if ((errors & UsartFlag::ne) != 0u) {
                    m_noise_errors = m_noise_errors + 1;
                }
            }

            const bool was_empty = m_rx.empty();
            const uint16_t fresh = RxEngine::take();
            if (fresh != 0u) {
                m_rx.publish(static_cast<typename decltype(m_rx)::index_t>(fresh));
            }
            // THE SILICON IS ASKED FIRST AND THE ARITHMETIC SECOND: a
            // channel that is not running gets a new run whatever the
            // count says. (The SAM campaign found a receive stream dead
            // in exactly the opposite rule.)
            if (RxEngine::idle() || RxEngine::full() || RxEngine::capacity() == 0u) {
                rearm_rx();
            }
            return was_empty && !m_rx.empty();
        }
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

    /**
     * @brief Move the LINK to a different bit rate, the clock staying put
     * - the mirror of rebase(), which moves the clock and keeps the rate.
     *
     * The queued bytes are drained at the OLD rate first (they were meant
     * for it), then BRR is written with the instance stopped, since it is
     * UE-protected like every other frame field. False, and nothing
     * written, when the new rate is unreachable at this clock.
     *
     * Main context only, and the caller owns the agreement with whatever
     * is on the other end: a receiver still at the old rate reads noise.
     */
    static bool set_baud(uint32_t hz, uint32_t baud) {
        const std::optional<uint16_t> reg = usart_brr(hz, baud);
        if (!reg) {
            return false;
        }
        constexpr uint32_t ring_drain_spins = 8'000'000u;
        constexpr uint32_t frame_spins = 200'000u;
        uint32_t spins = ring_drain_spins;
        while (!m_tx.empty() && spins-- != 0u) {
        }
        spins = frame_spins;
        while ((S::status() & UsartFlag::tc) == 0u && spins-- != 0u) {
        }
        S::enable(false);
        S::regs().BRR = *reg;
        S::enable(true);
        m_baud = baud;
        return true;
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

    /**
     * Queue one byte; false when the ring is full (print() retries).
     *
     * WITH AN ENGINE, A REFUSED BYTE STILL NUDGES. print() answers a
     * false by trying again for ever, so a path that can leave the
     * transport unpoked stops the program - and the ring is full
     * precisely when nothing is draining it. The plain transport does not
     * need it (a push that filled the ring already armed TXE, and TXE is
     * a condition that cannot be missed), which is also why the
     * engineless image is byte-identical to the one built before these
     * slots existed.
     */
    static bool write_byte(uint8_t b) {
        if (!m_tx.push(b)) {
            if constexpr (has_tx_engine) {
                pump_tx();
            }
            return false;
        }
        if constexpr (has_tx_engine) {
            pump_tx();
        } else {
            S::txe_interrupt(true);
        }
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
        // ONE nudge for the whole run - the entire point of the verb.
        if constexpr (has_tx_engine) {
            pump_tx();
        } else if (queued != 0u) {
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

    /// DMA blocks this transport threw away because the controller had
    /// stopped running them (a transfer error disables a channel in
    /// hardware, RM0444 10.4.7). Always 0, and free, without an engine.
    static uint8_t dma_faults() {
        if constexpr (has_tx_engine || has_rx_engine) {
            return m_dma_faults;
        } else {
            return 0;
        }
    }

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
        if constexpr (has_tx_engine) {
            TxEngine::stop();
        }
        if constexpr (has_rx_engine) {
            RxEngine::stop();
        }
        S::enable(false);
        S::reset();
        TxPin::release();
        RxPin::release();
        S::bus_clock(false);
        m_rx.clear();
        m_tx.clear();
    }

private:
    /**
     * Hand the transmit engine the ring's next contiguous run, if it is
     * free to take one.
     *
     * NO KICK, AND THAT IS THIS CONTROLLER'S OWN FACT. The SAM's twin had
     * to ask the peripheral whether its request was already standing and
     * software-trigger the channel if it was, because that DMAC latches a
     * trigger on the RISE. Here 10.4.3's handshake is level-driven: the
     * channel is enabled, it sees TXE asserted, it writes TDR. Measured
     * (test_stm32_dma letter h) rather than assumed, because a wrong
     * answer is a transmitter that never starts.
     */
    static void pump_tx() {
        if constexpr (has_tx_engine) {
            typename Stm32Platform::CriticalSection cs;
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

    /// Point the receive engine at the ring's next free run. A ring with
    /// no room at all is a byte lost before it arrives, and it is counted
    /// as the software overrun it is.
    static void rearm_rx() {
        if constexpr (has_rx_engine) {
            const auto room = m_rx.write_span();
            if (room.empty()) {
                m_rx_overruns = m_rx_overruns + 1;
                return;
            }
            (void)RxEngine::start(room.data(), static_cast<uint16_t>(room.size()));
        }
    }
};

} // namespace brio
