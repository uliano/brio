/*
 * spi.hpp
 *
 * The RP2040's two SPI controllers (datasheet 4.4): the ARM PL022
 * synchronous serial port - a host or a client, Motorola SPI, TI or
 * Microwire framing, frames of 4 to 16 bits, two 8-deep 16-bit FIFOs,
 * a bit rate of clk_peri over an even prescaler (2..254) times a
 * serial clock rate (1..256), a loop-back, four interrupt sources on
 * one line per instance, and a DMA request per FIFO. Three layers,
 * the other strata's arrangement:
 *
 *  - `Pl022<n>` is the RESOURCE: the register block behind the reset
 *    controller's gate, the configuration under the block's rule (the
 *    control registers with SSE clear), the data and status verbs, the
 *    interrupt trio through the atomic aliases, the ISR body that
 *    reads the raised-and-enabled sources. It decides nothing.
 *  - `SpiHost<n, pins, TxEngine, RxEngine>` is the ENGINE util/
 *    spi_bus.hpp's SpiBus drives: the Request is the other strata's
 *    VERBATIM (a chip select, a D/C line, a command phase, a data phase
 *    with optional out and in, the per-request mode / rate / frame
 *    size, the polled or ISR-pumped completion), so an application
 *    written over SpiBus on another target runs here unchanged.
 *  - `SpiClient<n, pins>` is the other end of the wire, a thin surface
 *    with an ISR body: a client is a protocol and which one is the
 *    application's.
 *
 * WHAT THIS SILICON HAS, AND HAS NOT. Two FIFOs of eight frames each,
 * so a host keeps up to eight frames in flight and the pump serves
 * them in batches: the receive interrupt at the FIFO's half (RXIM,
 * four frames) and the RECEIVE TIMEOUT (RTIM: a frame waiting and no
 * clock for 32 bit periods) deliver a short tail, the transmit
 * interrupt (TXIM, the transmit FIFO at or below half) is never the
 * pump's edge - a frame that came BACK is the one witness the bus is
 * idle for the next, the other strata's rule, and the pump writes
 * only while fewer than eight are in flight, so the receive FIFO
 * cannot overrun. The bit rate is clk_peri / (CPSDVSR x (1 + SCR)):
 * 62.5 Mbit/s at the top of a 125 MHz clk_peri for a host (CPSDVSR 2,
 * SCR 0), and a CLIENT needs clk_peri at least twelve times its clock
 * (4.4.3.4: the input is double-synchronized), 10.4 Mbit/s here. The
 * frame is any width from 4 to 16 bits (DSS); the Request speaks the
 * two the other strata know, 8 and 16, and the resource takes any.
 * Control registers are written with the port disabled (SSE clear):
 * a change of mode, rate or width costs a disable/enable pair, paid
 * only when something moved. The host's chip select is a GPIO the
 * Request carries (docs/design/spi-bus.md's arrangement on every
 * target): the PL022 would pulse its own SSPFSSOUT between frames in
 * Motorola format, which a device reading a multi-frame transaction
 * inside one select window cannot take. The CLIENT's select IS the
 * pad: the PL022 in slave mode frames on SSPFSSIN, so the client's CS
 * pin goes to the peripheral. LBM loops the transmitter into the
 * receiver with nothing on the wire: the wireless instrument of the
 * bench suite, a HOST verb (`SpiHost::loopback`) because the host
 * re-applies its whole configuration at a request that changes mode,
 * rate or width.
 *
 * TWO FACTS OF THE PL022 AS A CLIENT (measured, the suite's wire
 * letters). With SPH = 0 (modes 0 and 2) the slave frames on the
 * SELECT: it takes ONE frame per select window and ignores the rest
 * while the select stays low - a host that holds a GPIO select for a
 * whole transaction, this engine's arrangement and every device's
 * expectation, reaches a PL022 client for more than one frame only in
 * modes 1 and 3 (SPH = 1, the select may stay low across frames). And
 * SOD does not release the pad on this chip: the transmit line under
 * SOD reads as a driven low, not as the host's pull-up, so the dark
 * listener of `SpiClient::drive_output(false)` releases the PAD, the
 * other strata's way, and SOD stays a bit of the resource.
 *
 * THE PINS are fixed per instance under function 1 (2.19.2, table
 * 279): SPI0 receives on GPIO 0, 4, 16, 20, selects on 1, 5, 17, 21,
 * clocks on 2, 6, 18, 22 and transmits on 3, 7, 19, 23; SPI1 on 8,
 * 12, 24, 28 / 9, 13, 25, 29 / 10, 14, 26 / 11, 15, 27. `spi_pins_valid`
 * is that table.
 *
 * THE DMA ENGINES ride any two channels, told the instance's requests
 * (Dreq::spi{n}_tx / spi{n}_rx, table 119) at arm(): the data phase of
 * a request moves as one block each way, the command phase on the
 * pump, the completion the RECEIVE block's. The requests are raised
 * around the engines and nowhere else - a request standing with no
 * channel behind it banks credits the next block would spend on a
 * FIFO that has not got the data (rp2040/dma.hpp).
 */

#pragma once

#include <stdint.h>
#include <optional>
#include <type_traits>

#include "rp2040/device.hpp"

#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "rp2040/delay.hpp"
#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/resets.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

/// SPO and SPH as the four Motorola modes (the mode's bit 1 is the
/// polarity, bit 0 the phase).
enum class SpiMode : uint8_t { mode0 = 0, mode1 = 1, mode2 = 2, mode3 = 3 };
constexpr bool spi_mode_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 2u) != 0u; }
constexpr bool spi_mode_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 1u) != 0u; }

/// SSPCR0.FRF: the three framings.
enum class SpiFormat : uint8_t { motorola = 0, ti = 1, microwire = 2 };

/// The bit rate: clk_peri / (cpsdvsr x (1 + scr)), cpsdvsr even in
/// 2..254, scr 0..255 (4.4.3.6.1).
struct SpiClock {
    uint8_t cpsdvsr = 2;
    uint8_t scr = 7;
    constexpr bool valid() const { return cpsdvsr >= 2u && (cpsdvsr & 1u) == 0u; }
    constexpr uint32_t divisor() const { return static_cast<uint32_t>(cpsdvsr) * (static_cast<uint32_t>(scr) + 1u); }
    constexpr bool operator==(const SpiClock& o) const { return cpsdvsr == o.cpsdvsr && scr == o.scr; }
};

/// The named divisors the other strata's requests speak (clk_peri
/// over a power of two), and any even divisor up to 65024.
struct SpiClocks {
    static constexpr SpiClock div2{2, 0};
    static constexpr SpiClock div4{2, 1};
    static constexpr SpiClock div8{2, 3};
    static constexpr SpiClock div16{2, 7};
    static constexpr SpiClock div32{2, 15};
    static constexpr SpiClock div64{2, 31};
    static constexpr SpiClock div128{2, 63};
    static constexpr SpiClock div256{2, 127};
};

constexpr uint32_t spi_sck_hz(uint32_t pclk, SpiClock c) { return pclk / c.divisor(); }

/// The fastest setting whose SCK does not exceed `max_hz` at `pclk`;
/// nullopt when even 254 x 256 does not slow it enough - refused,
/// never rounded up.
constexpr std::optional<SpiClock> spi_clock_for(uint32_t pclk, uint32_t max_hz) {
    if (pclk == 0u || max_hz == 0u) {
        return {};
    }
    std::optional<SpiClock> best{};
    uint32_t best_hz = 0;
    for (uint32_t cpsdvsr = 2; cpsdvsr <= 254u; cpsdvsr += 2u) {
        // The smallest 1 + scr that keeps pclk / (cpsdvsr x (1 + scr)) <= max.
        const uint64_t need = (static_cast<uint64_t>(pclk) + static_cast<uint64_t>(cpsdvsr) * max_hz - 1u) /
                              (static_cast<uint64_t>(cpsdvsr) * max_hz);
        if (need > 256u) {
            continue;
        }
        const uint32_t scr1 = need == 0u ? 1u : static_cast<uint32_t>(need);
        const uint32_t hz = pclk / (cpsdvsr * scr1);
        if (hz > best_hz) {
            best_hz = hz;
            best = SpiClock{static_cast<uint8_t>(cpsdvsr), static_cast<uint8_t>(scr1 - 1u)};
        }
    }
    return best;
}

/// The two frame sizes the Request speaks (the resource takes 4..16).
enum class SpiDataSize : uint8_t { bits8 = 0, bits16 = 1 };
constexpr bool spi_frame_is_halfword(SpiDataSize s) { return s == SpiDataSize::bits16; }
constexpr uint8_t spi_data_bits(SpiDataSize s) { return s == SpiDataSize::bits16 ? 16u : 8u; }
constexpr uint16_t spi_frame_mask(SpiDataSize s) { return s == SpiDataSize::bits16 ? 0xFFFFu : 0x00FFu; }

enum class SpiRole : uint8_t { client = 0, host = 1 };

/// Which pins carry the four signals: each the instance's own under
/// function 1, or none (0xFF). The host's `cs` is NOT the peripheral's
/// (a GPIO the Request carries); the client's IS.
struct SpiPins {
    uint8_t sck = 0xFFu;
    uint8_t tx = 0xFFu;
    uint8_t rx = 0xFFu;
    uint8_t cs = 0xFFu;
};

constexpr bool spi_sck_pin(uint8_t n, uint8_t p) { return p < gpio_count && (p & 3u) == 2u && ((p >> 3) & 1u) == n; }
constexpr bool spi_tx_pin(uint8_t n, uint8_t p) { return p < gpio_count && (p & 3u) == 3u && ((p >> 3) & 1u) == n; }
constexpr bool spi_rx_pin(uint8_t n, uint8_t p) { return p < gpio_count && (p & 3u) == 0u && ((p >> 3) & 1u) == n; }
constexpr bool spi_cs_pin(uint8_t n, uint8_t p) { return p < gpio_count && (p & 3u) == 1u && ((p >> 3) & 1u) == n; }

/// A link needs SCK; every pin named must be the instance's own for
/// its signal; no two the same.
constexpr bool spi_pins_valid(uint8_t n, const SpiPins& p) {
    if (n > 1u || !spi_sck_pin(n, p.sck)) {
        return false;
    }
    if (p.tx != 0xFFu && !spi_tx_pin(n, p.tx)) { return false; }
    if (p.rx != 0xFFu && !spi_rx_pin(n, p.rx)) { return false; }
    if (p.cs != 0xFFu && !spi_cs_pin(n, p.cs)) { return false; }
    return true;
}

/// The whole configuration of the block.
struct SpiConfig {
    SpiRole role = SpiRole::host;
    SpiMode mode = SpiMode::mode0;
    SpiFormat format = SpiFormat::motorola;
    uint8_t bits = 8;                 ///< DSS + 1: 4..16
    SpiClock clock = SpiClocks::div16;
    bool loopback = false;            ///< LBM: the transmitter into the receiver
    bool output_disabled = false;     ///< SOD: a client that answers nothing on the wire
};

constexpr bool spi_config_valid(const SpiConfig& c) {
    return c.bits >= 4u && c.bits <= 16u && c.clock.valid() &&
           static_cast<uint8_t>(c.format) <= 2u &&
           !(c.role == SpiRole::host && c.output_disabled);
}

/// SSPCR0 for a configuration.
constexpr uint32_t spi_cr0_of(const SpiConfig& c) {
    uint32_t v = static_cast<uint32_t>(c.clock.scr) << SPI_SSPCR0_SCR_LSB;
    v |= static_cast<uint32_t>(c.format) << SPI_SSPCR0_FRF_LSB;
    v |= static_cast<uint32_t>(c.bits - 1u) << SPI_SSPCR0_DSS_LSB;
    if (c.format == SpiFormat::motorola) {
        if (spi_mode_cpol(c.mode)) { v |= SPI_SSPCR0_SPO_BITS; }
        if (spi_mode_cpha(c.mode)) { v |= SPI_SSPCR0_SPH_BITS; }
    }
    return v;
}

/// SSPCR1 for a configuration, SSE clear.
constexpr uint32_t spi_cr1_of(const SpiConfig& c) {
    uint32_t v = 0;
    if (c.role == SpiRole::client) { v |= SPI_SSPCR1_MS_BITS; }
    if (c.loopback) { v |= SPI_SSPCR1_LBM_BITS; }
    if (c.output_disabled) { v |= SPI_SSPCR1_SOD_BITS; }
    return v;
}

/// SSPSR's flags.
struct SpiFlag {
    static constexpr uint32_t tx_empty = SPI_SSPSR_TFE_BITS;
    static constexpr uint32_t tx_not_full = SPI_SSPSR_TNF_BITS;
    static constexpr uint32_t rx_not_empty = SPI_SSPSR_RNE_BITS;
    static constexpr uint32_t rx_full = SPI_SSPSR_RFF_BITS;
    static constexpr uint32_t busy = SPI_SSPSR_BSY_BITS;
};

/// The four interrupt sources: one layout for IMSC, RIS, MIS and ICR
/// (only the timeout and the overrun clear through ICR; the two FIFO
/// levels clear by moving data).
struct SpiInterrupt {
    static constexpr uint32_t tx = SPI_SSPIMSC_TXIM_BITS;          ///< the transmit FIFO at or below half
    static constexpr uint32_t rx = SPI_SSPIMSC_RXIM_BITS;          ///< the receive FIFO at or above half
    static constexpr uint32_t rx_timeout = SPI_SSPIMSC_RTIM_BITS;  ///< data waiting, 32 bit periods idle
    static constexpr uint32_t overrun = SPI_SSPIMSC_RORIM_BITS;    ///< the receive FIFO overflowed
    static constexpr uint32_t all = tx | rx | rx_timeout | overrun;
};

// =============================================================================
// The resource
// =============================================================================

template <uint8_t n>
struct Pl022 {
    static_assert(n < 2, "the RP2040 has SPI0 and SPI1");
    Pl022() = delete;

    static constexpr uint8_t index = n;
    static constexpr uint32_t reset_bit = n == 0 ? ResetBlock::spi0 : ResetBlock::spi1;
    static constexpr uint8_t fifo_depth = 8;
    static constexpr Dreq dreq_tx = n == 0 ? Dreq::spi0_tx : Dreq::spi1_tx;
    static constexpr Dreq dreq_rx = n == 0 ? Dreq::spi0_rx : Dreq::spi1_rx;

    static SPI0_Type& regs() { return *(n == 0 ? SPI0 : SPI1); }
    static constexpr IRQn_Type irq() { return n == 0 ? SPI0_IRQ_IRQn : SPI1_IRQ_IRQn; }
    static volatile void* data_address() { return &regs().SSPDR; }

    /// The block from its reset state (2.14): held, released, ready.
    static bool reset() { return Resets::cycle(reset_bit); }
    static bool released() { return Resets::released(reset_bit); }
    static void hold() { Resets::hold(reset_bit); }

    static bool enabled() { return (regs().SSPCR1 & SPI_SSPCR1_SSE_BITS) != 0u; }
    static void enable(bool on) {
        if (on) { hw_set(regs().SSPCR1, SPI_SSPCR1_SSE_BITS); }
        else { hw_clear(regs().SSPCR1, SPI_SSPCR1_SSE_BITS); }
    }

    /// The whole configuration, refused while enabled (the block's rule:
    /// control registers are programmed with SSE clear) and for what
    /// spi_config_valid refuses. The interrupt mask is kept.
    static bool configure(const SpiConfig& c) {
        if (enabled() || !spi_config_valid(c)) {
            return false;
        }
        regs().SSPCPSR = c.clock.cpsdvsr;
        regs().SSPCR0 = spi_cr0_of(c);
        regs().SSPCR1 = spi_cr1_of(c);
        return true;
    }

    /// LBM and SOD are UARTCR-style live bits: allowed under SSE.
    static void loopback(bool on) {
        if (on) { hw_set(regs().SSPCR1, SPI_SSPCR1_LBM_BITS); } else { hw_clear(regs().SSPCR1, SPI_SSPCR1_LBM_BITS); }
    }
    static bool loopback() { return (regs().SSPCR1 & SPI_SSPCR1_LBM_BITS) != 0u; }
    static void output_disabled(bool on) {
        if (on) { hw_set(regs().SSPCR1, SPI_SSPCR1_SOD_BITS); } else { hw_clear(regs().SSPCR1, SPI_SSPCR1_SOD_BITS); }
    }

    /// The rate as the registers hold it.
    static SpiClock clock() {
        return SpiClock{static_cast<uint8_t>(regs().SSPCPSR & SPI_SSPCPSR_CPSDVSR_BITS),
                        static_cast<uint8_t>((regs().SSPCR0 & SPI_SSPCR0_SCR_BITS) >> SPI_SSPCR0_SCR_LSB)};
    }
    static uint8_t bits() { return static_cast<uint8_t>((regs().SSPCR0 & SPI_SSPCR0_DSS_BITS) + 1u); }

    // ---- data and status ------------------------------------------------------
    static uint32_t flags() { return regs().SSPSR; }
    static bool tx_not_full() { return (flags() & SpiFlag::tx_not_full) != 0u; }
    static bool tx_empty() { return (flags() & SpiFlag::tx_empty) != 0u; }
    static bool rx_not_empty() { return (flags() & SpiFlag::rx_not_empty) != 0u; }
    static bool rx_full() { return (flags() & SpiFlag::rx_full) != 0u; }
    static bool busy() { return (flags() & SpiFlag::busy) != 0u; }

    static void write_data(uint16_t v) { regs().SSPDR = v; }
    static uint16_t read_data() { return static_cast<uint16_t>(regs().SSPDR & SPI_SSPDR_DATA_BITS); }
    /// Throw away whatever the receive FIFO holds.
    static void flush_rx() {
        while (rx_not_empty()) {
            (void)regs().SSPDR;
        }
    }

    // ---- interrupts -----------------------------------------------------------
    static void interrupts(uint32_t mask, bool on) {
        if (on) { hw_set(regs().SSPIMSC, mask); } else { hw_clear(regs().SSPIMSC, mask); }
    }
    static uint32_t interrupts() { return regs().SSPIMSC; }
    static uint32_t raw_pending() { return regs().SSPRIS; }
    static uint32_t pending() { return regs().SSPMIS; }
    /// The two clearable sources (the timeout, the overrun).
    static void clear_pending(uint32_t mask) { regs().SSPICR = mask & (SpiInterrupt::rx_timeout | SpiInterrupt::overrun); }

    /// The raised-and-enabled sources, the way the other strata's ISR
    /// bodies answer; the two clearable ones are cleared here.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t up = pending();
        if ((up & (SpiInterrupt::rx_timeout | SpiInterrupt::overrun)) != 0u) {
            clear_pending(up);
        }
        return up;
    }

    /// The DMA request enables (SSPDMACR).
    static void dma_requests(bool tx, bool rx) {
        regs().SSPDMACR = (tx ? SPI_SSPDMACR_TXDMAE_BITS : 0u) | (rx ? SPI_SSPDMACR_RXDMAE_BITS : 0u);
    }
};

// =============================================================================
// The host engine
// =============================================================================

/// A DMA block the engines could not finish, or a polled block that
/// never completed: the engine's own status code, in the range
/// util/bus_master.hpp leaves to engines.
inline constexpr uint8_t spi_dma_fault = bus_engine_status;

/**
 * SpiHost<n, pins, TxEngine, RxEngine>
 *
 * The engine SpiBus (util/spi_bus.hpp = BusMaster) drives. Its Request
 * is the other strata's (the file header): the bus AO asserts the
 * request's own chip select around the transaction, a COMMAND phase
 * goes out with D/C low and a DATA phase with it high, the data phase
 * has an optional out buffer (null = 0xFF dummies) and an optional in
 * buffer (null = the frames read back are discarded), and every buffer
 * is LENT until the reply lands. The mode, rate and frame size travel
 * per request; the bit order is the PL022's (most significant first,
 * no other) and no verb offers another.
 *
 * THE PUMP keeps up to eight frames in flight through the FIFOs: on
 * every receive interrupt (the FIFO's half, or the timeout for a tail)
 * it takes what came back and writes as many more as it took. The
 * phase boundary drains: the first data frame goes out only when the
 * last command frame has come back, so D/C flips between them on the
 * wire. Two completion styles, the request's choice: `polled` false
 * runs on the interrupt with the kernel free between batches, `polled`
 * true spins inside start() with this SPI's own line silenced.
 *
 * THE ENGINE SLOTS: a DmaTxEngine and a DmaRxEngine on any two
 * channels, both or neither (the data phase is full duplex and the
 * transaction's completion is the RECEIVE block's). A request in
 * 16-bit frames falls back to the pump: the engines carry bytes.
 *
 * FRAMES IN A BYTE BUFFER: one byte per 8-bit frame, two bytes
 * low-first per 16-bit frame - the other strata's rule.
 */
template <uint8_t n, SpiPins pins, typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
class SpiHost {
    using S = Pl022<n>;
    static_assert(spi_pins_valid(n, pins),
                  "brio SpiHost: these pins cannot carry this SPI (2.19.2, table 279: under function "
                  "1 SPI0 clocks on GPIO 2, 6, 18, 22, transmits on 3, 7, 19, 23, receives on 0, 4, "
                  "16, 20; SPI1 on 10, 14, 26 / 11, 15, 27 / 8, 12, 24, 28)");
    static_assert(pins.tx != 0xFFu, "brio SpiHost: a host needs SCK and TX; RX is optional for a write-only bus");
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0, "the engine slots must name a complete type");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is full-duplex and "
                  "its completion is the RECEIVE block's");
    static_assert(uart_engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the two engines must ride two different DMA channels");
    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio SpiHost: the DMA engines must carry uint8_t elements - the Request's buffers are "
         "bytes and the engined path serves 8-bit frames");

    using SckPin = Pin<pins.sck>;
    using TxPin = Pin<pins.tx>;
    using RxPin = Pin<pins.rx != 0xFFu ? pins.rx : pins.sck>;

public:
    SpiHost() = delete;
    using Resource = S;
    static constexpr SpiPins pin_set = pins;
    static constexpr bool has_engines = TxEngine::present;

    /// The loop-back (LBM) as part of the applied configuration, so a
    /// request that re-applies mode, rate or width keeps it: the
    /// wireless instrument. The bus must be idle.
    static void loopback(bool on) {
        if (applied_.loopback == on) {
            return;
        }
        applied_.loopback = on;
        S::enable(false);
        (void)S::configure(applied_);
        S::enable(true);
        S::flush_rx();
    }
    static bool loopback() { return applied_.loopback; }

    struct Request {
        PinRef cs;   ///< asserted low around the transaction
        PinRef dc;   ///< display D/C line; null = no such pin
        /// Microseconds between the CS assertion and the first clock -
        /// what a device's datasheet calls CS setup. Spent spinning in
        /// start(), main context; 0 = none.
        uint8_t cs_setup_us = 0;
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        uint8_t cmd_len;   ///< in FRAMES
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len;      ///< phase 2 length, in FRAMES
        ReplyTo<SpiDone> reply;
        /// Per-transaction bus configuration: a shared bus's devices
        /// each name their own, and the engine reprograms the peripheral
        /// only when something CHANGED.
        SpiClock clock = SpiClocks::div16;
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;
        /// Completion style: false = the ISR pump, true = POLLED inside
        /// start().
        bool polled = false;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the instance up as a host: the block out of reset, the boot
    /// configuration, the pads, the line. `clock` is the app's Clock tag
    /// - the divisor is of Clock::pclk_hz, clk_peri. `max_sck_hz` is an
    /// optional CEILING for the whole bus; 0 = none. False when the
    /// ceiling cannot be produced or the block did not come up.
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not list it "
                      "among its Users: its SCK ceiling and its cs_setup timing would go stale");
        (void)clock;
        Nvic::disable(S::irq());
        ceiling_hz_ = max_sck_hz;
        rebase(Clock::pclk_hz, clock_hz(clock));
        if (max_sck_hz != 0u && !ceiling_) {
            return false;
        }
        if (!S::reset()) {
            return false;
        }
        applied_ = boot_config();
        if (!S::configure(applied_)) {
            return false;
        }
        status_ = spi_ok;
        S::interrupts(SpiInterrupt::all, false);
        if constexpr (has_engines) {
            TxEngine::arm(S::data_address(), S::dreq_tx);
            RxEngine::arm(S::data_address(), S::dreq_rx);
        }
        // The pads go to the peripheral with the registers already
        // holding the idle polarity, then SSE: a pad handed over first
        // would show whatever an unconfigured block drives.
        SckPin::function(PinFunction::spi);
        TxPin::function(PinFunction::spi);
        if constexpr (pins.rx != 0xFFu) {
            RxPin::function(PinFunction::spi, {.pull = PinPull::up});
        }
        S::enable(true);
        S::flush_rx();
        Nvic::enable(S::irq());
        return true;
    }

    /// clk_peri and clk_sys changed. A Request's `clock` is a DIVISION
    /// of clk_peri and scales by itself; what is recomputed is the
    /// ceiling and the cs_setup timing. THE BUS MUST BE IDLE.
    static void rebase(uint32_t pclk_hz, uint32_t sys_hz) {
        pclk_hz_ = pclk_hz;
        ceiling_ = ceiling_hz_ != 0u ? spi_clock_for(pclk_hz, ceiling_hz_) : std::optional<SpiClock>{};
        cs_rate_ = delay_rate(sys_hz);
    }
    /// The setting that produces at most `hz` of SCK at the clk_peri
    /// last seen - the chooser a device's datasheet limit is spoken to.
    static std::optional<SpiClock> clock_for(uint32_t hz) { return spi_clock_for(pclk_hz_, hz); }
    /// What a request at this setting really runs at, ceiling included.
    static uint32_t sck_hz(SpiClock c) { return spi_sck_hz(pclk_hz_, clamp(c)); }
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    static uint32_t reference_hz() { return pclk_hz_; }

    /// Put the peripheral at this mode, rate and frame size NOW, moving
    /// no data - for callers that frame the select window themselves
    /// (Request.cs null). A mode change is a CPOL FLIP ON THE WIRE and a
    /// selected client counts it as an edge: prime FIRST, then select.
    static void prime(SpiMode m, SpiClock c, SpiDataSize bits = SpiDataSize::bits8) {
        apply(m, clamp(c), bits);
    }

    // ---- the transfer -------------------------------------------------------

    /// Begin a transaction (SpiBus calls it from main context). True when
    /// it completed SYNCHRONOUSLY (polled requests, the empty one); false
    /// when it runs on the ISR and a TransferDone follows - exactly
    /// util/bus_master.hpp's engine contract.
    static bool start(const Request& r) {
        req_ = r;
        sent_ = 0;
        received_ = 0;
        in_cmd_ = (r.cmd_len > 0u);
        status_ = spi_ok;
        if (total_len() == 0u) {
            return true;
        }
        apply(r.mode, clamp(r.clock), r.bits);
        if (in_cmd_) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();   // assert, active low
        if (r.cs_setup_us != 0u) {
            (void)delay_us(cs_rate_, r.cs_setup_us);
        }
        S::flush_rx();   // a stale frame would be captured as this one's
        if constexpr (has_engines) {
            if (dma_serves(r)) {
                if (!r.polled) {
                    if (in_cmd_) {
                        // The command phase runs on the pump; isr()
                        // hands over to the engines at its end.
                        S::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, true);
                        fill();
                        return false;
                    }
                    launch_dma();
                    return false;   // dma_isr() is the completion edge
                }
                S::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, false);
                for (uint8_t i = 0; i < r.cmd_len; ++i) {
                    (void)xfer(frame_at(r.cmd.get(), i));
                }
                r.dc.set();
                if (r.len != 0u) {
                    launch_dma();
                    spin_dma();
                }
                r.cs.set();
                return true;
            }
        }
        if (!r.polled) {
            S::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, true);
            fill();   // the ISR pumps the rest
            return false;
        }
        // Polled pump: silence this SPI's own line (a bound handler would
        // steal the frames). Global interrupts STAY ENABLED.
        S::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, false);
        while (received_ < total_len()) {
            fill();
            uint32_t spins = 200'000u;
            while (!S::rx_not_empty() && spins-- != 0u) {
            }
            if (!S::rx_not_empty()) {
                break;
            }
            take();
        }
        r.cs.set();
        return true;
    }

    /// The instance's interrupt body - call from its vector. The receive
    /// level and the timeout are the pump's edges; the batch that came
    /// back is taken and the next written. True when the transaction
    /// just completed (CS released): the edge the app's glue posts
    /// TransferDone on.
    [[gnu::always_inline]] static bool isr() {
        const uint32_t up = S::isr();
        if ((up & (SpiInterrupt::rx | SpiInterrupt::rx_timeout)) == 0u) {
            return false;
        }
        take();
        if constexpr (has_engines) {
            if (dma_serves(req_) && !in_cmd_ && received_ >= req_.cmd_len && !dma_active_) {
                // The command phase is done on the pump: the engines take
                // the data phase, or the transaction is over.
                S::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, false);
                if (req_.len == 0u) {
                    req_.cs.set();
                    return true;
                }
                launch_dma();
                return false;
            }
        }
        if (received_ >= total_len()) {
            S::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, false);
            req_.cs.set();
            return true;
        }
        fill();
        return false;
    }

    /// The DMA line's interrupt body - call it from the line the engines
    /// report on. A bus error on either channel ends the transaction
    /// with spi_dma_fault. Compiles away on an engineless host. True
    /// when the transaction just completed.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint8_t tx = TxEngine::service();
            if ((tx & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                return finish_dma(spi_dma_fault);
            }
            if ((tx & TxEngine::flag_complete) != 0u) {
                (void)TxEngine::complete();   // the transmit side never completes a transaction
            }
            const uint8_t rx = RxEngine::service();
            if (rx != 0u && dma_active_) {
                if ((rx & RxEngine::flag_error) != 0u) {
                    return finish_dma(spi_dma_fault);
                }
                if ((rx & RxEngine::flag_complete) != 0u) {
                    return finish_dma(spi_ok);
                }
            }
        }
        return false;
    }

    /// The engine's completion status, for the TransferDone payload.
    static uint8_t status() { return status_; }

    /// Put the ENGINE back where start() is legal (the verb a timed
    /// SpiBus calls on a transaction that never answered): the select
    /// released FIRST, the engines put away and re-claimed, the block
    /// reset and reconfigured to the applied state.
    static bool recover() {
        req_.cs.set();
        if constexpr (has_engines) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address(), S::dreq_rx);
        }
        Nvic::disable(S::irq());
        in_cmd_ = false;
        dma_active_ = false;
        dma_done_ = false;
        S::enable(false);
        const bool ok = S::reset();
        const bool cfg = S::configure(applied_);
        S::interrupts(SpiInterrupt::all, false);
        S::enable(true);
        S::flush_rx();
        Nvic::enable(S::irq());
        return ok && cfg;
    }

    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Nvic::disable(S::irq());
        S::interrupts(SpiInterrupt::all, false);
        S::enable(false);
        SckPin::release();
        TxPin::release();
        if constexpr (pins.rx != 0xFFu) {
            RxPin::release();
        }
        S::hold();
    }

private:
    /// The engines serve a request in 8-bit frames; 16-bit ones are two
    /// bytes a datum, which byte engines cannot express.
    static bool dma_serves(const Request& r) { return !spi_frame_is_halfword(r.bits); }

    static void launch_dma() {
        dma_done_ = false;
        dma_active_ = true;
        // THE REQUESTS ARE RAISED AROUND THE ENGINES AND NOWHERE ELSE (the
        // file header). The receive channel first: its request rises only
        // when a frame has come back, and the transmit side is what
        // starts the clock.
        if (req_.rx.get() != nullptr) {
            (void)RxEngine::start(req_.rx.get(), req_.len);
        } else {
            (void)RxEngine::start_discard(&rx_sink_, req_.len);
        }
        S::dma_requests(false, true);
        if (req_.tx.get() != nullptr) {
            (void)TxEngine::start(req_.tx.get(), req_.len);
        } else {
            (void)TxEngine::start_fixed(&tx_dummy_, req_.len);
        }
        S::dma_requests(true, true);
    }

    /// One exit for the data phase. True when the ISR-style caller
    /// should post completion.
    static bool finish_dma(uint8_t st) {
        S::dma_requests(false, false);
        if (st != spi_ok) {
            status_ = st;
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address(), S::dreq_rx);
        } else if (TxEngine::busy()) {
            // The receive block completing is proof the transmit one did.
            (void)TxEngine::complete();
        }
        dma_active_ = false;
        dma_done_ = true;
        if (!req_.polled) {
            req_.cs.set();
            return true;
        }
        return false;
    }

    /// The polled request's wait on the DMA completion - bounded (the
    /// slowest frame is 65024 x 16 clk_peri cycles; the budget scales).
    static void spin_dma() {
        uint32_t spins = 200'000u + 20'000u * static_cast<uint32_t>(req_.len);
        while (!dma_done_ && spins-- != 0u) {
        }
        if (!dma_done_) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address(), S::dreq_rx);
            dma_active_ = false;
            status_ = spi_dma_fault;
        }
    }

    /// A slower rate is a LARGER divisor, so the ceiling clamps from below.
    static SpiClock clamp(SpiClock c) {
        if (!ceiling_ || c.divisor() >= ceiling_->divisor()) {
            return c;
        }
        return *ceiling_;
    }

    static SpiConfig boot_config() {
        SpiConfig c{};
        c.role = SpiRole::host;
        c.mode = SpiMode::mode0;
        c.bits = 8;
        c.clock = ceiling_ ? *ceiling_ : SpiClocks::div16;
        return c;
    }

    /// Put the peripheral where this request wants it: the control
    /// registers are written with SSE clear, so any change costs a
    /// disable/enable pair - paid only when something moved.
    static void apply(SpiMode m, SpiClock c, SpiDataSize bits) {
        const uint8_t width = spi_data_bits(bits);
        if (m == applied_.mode && c == applied_.clock && width == applied_.bits) {
            return;
        }
        applied_.mode = m;
        applied_.clock = c;
        applied_.bits = width;
        S::enable(false);
        (void)S::configure(applied_);
        S::enable(true);
    }

    /// Write frames while the transmit FIFO takes them and fewer than
    /// eight are in flight; the data phase waits for the command phase
    /// to come back whole (the D/C flip on the wire).
    static void fill() {
        while (S::tx_not_full() && sent_ - received_ < S::fifo_depth) {
            if (in_cmd_) {
                if (sent_ >= req_.cmd_len) {
                    return;   // the command frames are out; the data waits for them back
                }
                S::write_data(frame_at(req_.cmd.get(), sent_));
            } else {
                if (sent_ >= total_len()) {
                    return;
                }
                S::write_data(data_frame(static_cast<uint16_t>(sent_ - req_.cmd_len)));
            }
            ++sent_;
        }
    }

    /// Take what came back: command echoes discarded, data stored, the
    /// phase boundary crossed when the last command frame is in.
    static void take() {
        while (S::rx_not_empty()) {
            const uint16_t in = S::read_data();
            if (received_ >= req_.cmd_len && req_.rx.get() != nullptr) {
                store_frame(req_.rx.get(), static_cast<uint16_t>(received_ - req_.cmd_len), in);
            }
            ++received_;
            if (in_cmd_ && received_ >= req_.cmd_len) {
                in_cmd_ = false;
                req_.dc.set();
            }
        }
    }

    /// One polled frame: write, spin on the return, read back. Bounded.
    static uint16_t xfer(uint16_t out) {
        S::write_data(out);
        uint32_t spins = 200'000u;
        while (!S::rx_not_empty() && spins-- != 0u) {
        }
        return S::read_data();
    }

    static uint16_t total_len() { return static_cast<uint16_t>(req_.cmd_len) + req_.len; }
    static uint16_t frame_at(const uint8_t* p, uint16_t i) {
        if (p == nullptr) {
            return 0xFFFFu;
        }
        if (spi_frame_is_halfword(req_.bits)) {
            const uint16_t k = static_cast<uint16_t>(2u * i);
            return static_cast<uint16_t>(p[k] | (static_cast<uint16_t>(p[k + 1u]) << 8));
        }
        return p[i];
    }
    static void store_frame(uint8_t* p, uint16_t i, uint16_t v) {
        if (spi_frame_is_halfword(req_.bits)) {
            const uint16_t k = static_cast<uint16_t>(2u * i);
            p[k] = static_cast<uint8_t>(v);
            p[k + 1u] = static_cast<uint8_t>(v >> 8);
        } else {
            p[i] = static_cast<uint8_t>(v);
        }
    }
    static uint16_t data_frame(uint16_t i) {
        return (req_.tx.get() != nullptr) ? frame_at(req_.tx.get(), i) : 0xFFFFu;
    }

    static inline Request req_{};
    static inline uint16_t sent_ = 0;
    static inline uint16_t received_ = 0;
    static inline bool in_cmd_ = false;
    static inline uint8_t status_ = spi_ok;
    static inline volatile bool dma_done_ = false;
    static inline volatile bool dma_active_ = false;
    static inline uint8_t rx_sink_ = 0;
    static constexpr uint8_t tx_dummy_ = 0xFF;
    static inline SpiConfig applied_{};
    static inline uint32_t pclk_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<SpiClock> ceiling_{};
    static inline DelayRate cs_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/**
 * SpiClient<n, pins>
 *
 * The other end of the wire: SCK, TX-in and the select are inputs, the
 * host sets the pace, and the only thing this side controls is WHAT it
 * has ready to shift out when the next clock arrives - up to EIGHT
 * frames ahead in the transmit FIFO (`frames_ahead`), written on TNF.
 * A client that falls behind does not stall the bus: what the host
 * reads is whatever the shifter held, a wrong answer and never a
 * missing one. THE SELECT IS THE PAD: the PL022 in slave mode frames
 * on SSPFSSIN, so `pins.cs` goes to the peripheral and `selected()`
 * reads the same pad. `drive_output(false)` is SOD: the transmit line
 * left undriven, a DARK LISTENER on a shared harness, the pad
 * untouched.
 */
template <uint8_t n, SpiPins pins>
class SpiClient {
    using S = Pl022<n>;
    static_assert(spi_pins_valid(n, pins), "brio SpiClient: these pins cannot carry this SPI (table 279)");
    static_assert(pins.rx != 0xFFu && pins.cs != 0xFFu,
                  "brio SpiClient: a client listens on RX and frames on its CS pad");
    using SckPin = Pin<pins.sck>;
    using RxPin = Pin<pins.rx>;
    using CsPin = Pin<pins.cs>;
    using TxPin = Pin<pins.tx != 0xFFu ? pins.tx : pins.sck>;

public:
    SpiClient() = delete;
    using Resource = S;
    static constexpr SpiPins pin_set = pins;

    struct Config {
        SpiMode mode = SpiMode::mode0;
        SpiFormat format = SpiFormat::motorola;
        uint8_t bits = 8;             ///< 4..16
        /// Whether the transmit line is driven at all (SOD clear).
        bool drive_output = true;
    };

    /// Bring the instance up as a client. `clock` is the app's Clock tag:
    /// clk_peri must run at least twelve times the host's clock
    /// (4.4.3.4), which is the caller's to know. False when the block
    /// did not come up or the configuration is refused.
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        Nvic::disable(S::irq());
        if (!S::reset()) {
            return false;
        }
        SpiConfig c{};
        c.role = SpiRole::client;
        c.mode = cfg.mode;
        c.format = cfg.format;
        c.bits = cfg.bits;
        if (!S::configure(c)) {
            return false;
        }
        bits_ = cfg.bits;
        S::interrupts(SpiInterrupt::all, false);
        // Inputs first, the output last.
        SckPin::function(PinFunction::spi);
        RxPin::function(PinFunction::spi);
        CsPin::function(PinFunction::spi, {.pull = PinPull::up});
        drive_output(cfg.drive_output);
        S::flush_rx();
        Nvic::enable(S::irq());
        return true;
    }

    /// The answer line handed to the peripheral, or taken back as an
    /// undriven input - the dark listener (the file header: SOD does not
    /// release the pad on this chip).
    static void drive_output(bool on) {
        if constexpr (pins.tx != 0xFFu) {
            if (on) { TxPin::function(PinFunction::spi); } else { TxPin::release(); }
        } else {
            (void)on;
        }
    }
    /// SOD, the block's own slave-output-disable bit, for a program that
    /// wants to see what it does on its board.
    static void sod(bool on) { S::output_disabled(on); }

    /// SSE up with the FIRST ANSWER in the FIFO before the host's clock
    /// can arrive. Call it with the select still high.
    static void enable(uint16_t first = 0xFFFFu) {
        S::enable(true);
        write(first);
    }
    static void disable() { S::enable(false); }

    /// How many answers may be queued ahead of the frame the host is
    /// about to clock: the transmit FIFO's depth.
    static constexpr uint8_t frames_ahead = S::fifo_depth;

    /// Load the next frame to shift out (on TNF).
    static void write(uint16_t v) { S::write_data(v); }
    static bool writable() { return S::tx_not_full(); }
    /// One received frame, or nothing.
    static std::optional<uint16_t> poll() {
        if (!S::rx_not_empty()) {
            return {};
        }
        return static_cast<uint16_t>(S::read_data() & ((1u << bits_) - 1u));
    }
    /// Is the host holding the select low right now? A live pad read.
    static bool selected() { return !CsPin::read(); }

    static bool overrun() { return (S::raw_pending() & SpiInterrupt::overrun) != 0u; }
    static void clear_overrun() { S::clear_pending(SpiInterrupt::overrun); }
    static uint32_t flags() { return S::flags(); }

    /// The ISR body: the raised-and-enabled sources, for the app's glue.
    [[gnu::always_inline]] static uint32_t isr() { return S::isr(); }
    static void interrupts(uint32_t mask, bool on) { S::interrupts(mask, on); }

    static void release() {
        Nvic::disable(S::irq());
        S::interrupts(SpiInterrupt::all, false);
        S::enable(false);
        SckPin::release();
        RxPin::release();
        CsPin::release();
        if constexpr (pins.tx != 0xFFu) {
            TxPin::release();
        }
        S::hold();
    }

private:
    static inline uint8_t bits_ = 8;
};

} // namespace brio
