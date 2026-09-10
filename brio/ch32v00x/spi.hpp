/*
 * spi.hpp
 *
 * The SPI of the CH32V00x (RM ch. 16): one instance, both roles, 8- or
 * 16-bit frames, the four modes, a software or hardware chip select,
 * hardware CRC, two DMA requests - the STM32F1's SPI with one register
 * of its own (HSCR, a "high-speed read" mode) and without the I2S half.
 * Three layers, the other three strata's arrangement:
 *
 *  - `Spi<n>` is the RESOURCE: the register block, its gate and reset,
 *    the configuration under the rules the chapter states, the data
 *    and status verbs, the ISR body that reads the raised-and-enabled
 *    sources. It decides nothing.
 *  - `SpiHost<n, pins, TxEngine, RxEngine>` is the ENGINE util/
 *    spi_bus.hpp's SpiBus drives: the Request is the other strata's
 *    VERBATIM (chip select, D/C, a command phase, a data phase with
 *    optional out and in, the per-request mode/rate/frame size, the
 *    polled or ISR-pumped completion), so an application written over
 *    SpiBus on a Nucleo runs here unchanged.
 *  - `SpiClient<n, pins>` is the other end of the wire, a thin polled
 *    surface with an ISR body: a client is a protocol and which one is
 *    the application's.
 *
 * WHAT THIS SILICON HAS, AND HAS NOT. There is NO FIFO: one transmit
 * buffer, one receive buffer, one shift register (figure 16-1), so TXE
 * means "the buffer moved into the shifter" and RXNE "a frame has
 * been shifted both ways" - and the host's pump runs on RXNE alone,
 * the way the other strata's do, because a frame that came back is the
 * one witness that the bus is idle for the next. The frame is 8 or 16
 * bits (DFF), and both DFF and CRCEN "can only be written when SPE is
 * 0" (16.3.1), which is what makes apply() pay a disable/enable pair
 * when a request changes the size. The baud rate is HCLK over a power
 * of two, /2 to /256, and the manual promises SCK up to HCLK/2 - 24 MHz
 * at the 48 MHz this stratum runs at. The chip select the engine uses
 * is a GPIO the Request carries (SSM set, SSI high): 16.2.1 makes the
 * NSS pad free under software management, and a select the bus AO
 * drives is the arrangement docs/design/spi-bus.md gives every target.
 *
 * TWO OF THE CHAPTER'S SENTENCES ARE SLIPS and this file reads past
 * them: DFF's two codes are both described as "16-bit" (0 is 8-bit,
 * the F1 lineage and the datasheet's "8-bit or 16-bit"), and 16.2.1's
 * description of CPHA says "the CPHA setting" for both values (1
 * samples on the second edge, 0 on the first - table 16-1, and the four
 * figures that follow). The DMA REQUESTS ARE CHANNELS 2 (receive) and
 * 3 (transmit), table 8-2: an engine slot naming any other channel is
 * refused at compile time, because on this family the channel IS the
 * request and a wrong one moves nothing.
 *
 * THE DISABLE PROCEDURE (16.2.7 for MODF, and the F1 lineage's rule
 * this chapter does not spell): clearing SPE while a frame is in
 * flight truncates it on the wire. `Spi::disable()` waits for RXNE
 * (the last frame back), TXE and not BSY before it drops the bit, each
 * wait bounded, and reports whether the waits ran out. Clearing MODF is
 * a STATR read followed by a CTLR1 write; clearing OVR a DATAR read
 * followed by a STATR read - two sequences the resource keeps.
 *
 * NOT COVERED YET: the simplex modes (BIDIMODE/BIDIOE, RXONLY) beyond
 * the resource's configuration bits - no user; the hardware NSS
 * arrangements as the engine's select (the resource has them, the
 * engine's select is a GPIO on purpose); the CRC beyond the resource's
 * verbs - born with a device that checks one; HSCR's high-speed read
 * mode, whose rate formula (16.3.1's note: HCLK/(BR+2)) the bench has
 * not measured. The pads come with their remap code (afio.hpp, table
 * 7-12): spi1_pins_for(code) is a whole column, and init() writes the
 * code.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "ch32v00x/afio.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/device.hpp"
#include "ch32v00x/dma_engine.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// =============================================================================
// The registers (table 16-2): sixteen bits at a four-byte stride
// =============================================================================

struct SpiRegs {
    volatile uint16_t CTLR1;   ///< 0x00
    uint16_t RESERVED0;
    volatile uint16_t CTLR2;   ///< 0x04
    uint16_t RESERVED1;
    volatile uint16_t STATR;   ///< 0x08
    uint16_t RESERVED2;
    volatile uint16_t DATAR;   ///< 0x0c
    uint16_t RESERVED3;
    volatile uint16_t CRCR;    ///< 0x10 the polynomial
    uint16_t RESERVED4;
    volatile uint16_t RCRCR;   ///< 0x14 receive CRC, read-only
    uint16_t RESERVED5;
    volatile uint16_t TCRCR;   ///< 0x18 transmit CRC, read-only
    uint16_t RESERVED6;
    uint32_t RESERVED7[2];     ///< 0x1c, 0x20 (the F1's I2S registers, absent here)
    volatile uint16_t HSCR;    ///< 0x24 high-speed control
    uint16_t RESERVED8;
};

inline constexpr uint32_t spi1_base = pb2_base + 0x3000;

constexpr uint32_t spi_base_for(uint8_t instance) {
    return instance == 1 ? spi1_base : 0;
}

// CTLR1 (16.3.1)
inline constexpr uint16_t spi_cpha     = 1u << 0;
inline constexpr uint16_t spi_cpol     = 1u << 1;
inline constexpr uint16_t spi_mstr     = 1u << 2;
inline constexpr uint16_t spi_br_mask  = 7u << 3;
inline constexpr uint16_t spi_spe      = 1u << 6;
inline constexpr uint16_t spi_lsbfirst = 1u << 7;
inline constexpr uint16_t spi_ssi      = 1u << 8;
inline constexpr uint16_t spi_ssm      = 1u << 9;
inline constexpr uint16_t spi_rxonly   = 1u << 10;
inline constexpr uint16_t spi_dff      = 1u << 11;
inline constexpr uint16_t spi_crcnext  = 1u << 12;
inline constexpr uint16_t spi_crcen    = 1u << 13;
inline constexpr uint16_t spi_bidioe   = 1u << 14;
inline constexpr uint16_t spi_bidimode = 1u << 15;
// CTLR2 (16.3.2)
inline constexpr uint16_t spi_rxdmaen  = 1u << 0;
inline constexpr uint16_t spi_txdmaen  = 1u << 1;
inline constexpr uint16_t spi_ssoe     = 1u << 2;
inline constexpr uint16_t spi_errie    = 1u << 5;
inline constexpr uint16_t spi_rxneie   = 1u << 6;
inline constexpr uint16_t spi_txeie    = 1u << 7;
// STATR (16.3.3)
inline constexpr uint16_t spi_rxne     = 1u << 0;
inline constexpr uint16_t spi_txe      = 1u << 1;
inline constexpr uint16_t spi_crcerr   = 1u << 4;
inline constexpr uint16_t spi_modf     = 1u << 5;
inline constexpr uint16_t spi_ovr      = 1u << 6;
inline constexpr uint16_t spi_bsy      = 1u << 7;
// HSCR (16.3.8)
inline constexpr uint16_t spi_hsrxen   = 1u << 0;

// =============================================================================
// The vocabulary
// =============================================================================

/// The flags STATR raises, as the ISR body hands them back.
struct SpiFlag {
    static constexpr uint32_t rxne = spi_rxne;
    static constexpr uint32_t txe = spi_txe;
    static constexpr uint32_t crc_error = spi_crcerr;
    static constexpr uint32_t mode_fault = spi_modf;
    static constexpr uint32_t overrun = spi_ovr;
    static constexpr uint32_t busy = spi_bsy;
};

/// Table 16-1: CPOL and CPHA as the four Motorola modes.
enum class SpiMode : uint8_t { mode0 = 0, mode1 = 1, mode2 = 2, mode3 = 3 };

constexpr bool spi_mode_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 2u) != 0u; }
constexpr bool spi_mode_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 1u) != 0u; }

/// BR[2:0]: HCLK over two to the code plus one.
enum class SpiClock : uint8_t {
    div2 = 0, div4 = 1, div8 = 2, div16 = 3, div32 = 4, div64 = 5, div128 = 6, div256 = 7,
};

constexpr uint32_t spi_sck_hz(uint32_t hclk, SpiClock c) {
    return hclk >> (static_cast<uint8_t>(c) + 1u);
}

/// The coarsest code whose SCK does not exceed `max_hz`; nullopt when
/// even HCLK/256 does - refused, never rounded up.
constexpr std::optional<SpiClock> spi_rate_for(uint32_t hclk, uint32_t max_hz) {
    if (hclk == 0u || max_hz == 0u) {
        return {};
    }
    for (uint8_t code = 0; code < 8u; ++code) {
        if ((hclk >> (code + 1u)) <= max_hz) {
            return static_cast<SpiClock>(code);
        }
    }
    return {};
}

/// DFF: the two frame sizes this family has.
enum class SpiDataSize : uint8_t { bits8 = 0, bits16 = 1 };

constexpr bool spi_frame_is_halfword(SpiDataSize s) { return s == SpiDataSize::bits16; }
constexpr uint8_t spi_data_bits(SpiDataSize s) { return s == SpiDataSize::bits16 ? 16u : 8u; }
constexpr uint16_t spi_frame_mask(SpiDataSize s) { return s == SpiDataSize::bits16 ? 0xFFFFu : 0x00FFu; }

enum class SpiRole : uint8_t { client = 0, host = 1 };

/// The chip select arrangements of 16.2.1: `software` is SSM/SSI (the
/// pad free, the engine's select a GPIO); `hardware_output` is SSOE, the
/// pad driven low by the host while SPE is set; `hardware_input` is the
/// multi-host input whose low level raises MODF and demotes the host.
enum class SpiNss : uint8_t { software, hardware_output, hardware_input };

/// 16.2.4's line arrangements.
enum class SpiDirection : uint8_t {
    full_duplex,       ///< two data lines, both ways
    receive_only,      ///< RXONLY: two lines, the output one released
    half_duplex_out,   ///< BIDIMODE + BIDIOE: one line, transmitting
    half_duplex_in,    ///< BIDIMODE, BIDIOE clear: one line, receiving
};

/// Which pads carry the four signals, and the REMAP CODE that puts the
/// peripheral on them (afio.hpp's table 7-12): init() writes the code
/// into AFIO_PCFR1. No alternate-function number - a pad has the one
/// function its column gives it. A signal the program does not wire is
/// an invalid Pad.
struct SpiPins {
    Pad sck{};
    Pad mosi{};
    Pad miso{};
    Pad nss{};
    uint8_t remap = 0;
};

/// The pins of remap column `code`, every signal named.
constexpr SpiPins spi1_pins_for(uint8_t code) {
    const SpiPadSet p = afio_spi1_pads(code);
    return SpiPins{.sck = p.sck, .mosi = p.mosi, .miso = p.miso, .nss = p.nss, .remap = code};
}

/// SPI1's default pads on the CH32V006 (DS table 2-1-1): SCK PC5, MOSI
/// PC6, MISO PC7, NSS PC1.
inline constexpr SpiPins spi1_default_pins = spi1_pins_for(0);

/// A link needs SCK, no two signals on one pad, a remap code the table
/// has.
constexpr bool spi_pins_valid(const SpiPins& p) {
    if (!p.sck.valid() || p.remap >= afio_spi1_codes) {
        return false;
    }
    const Pad pads[] = {p.sck, p.mosi, p.miso, p.nss};
    for (uint8_t i = 0; i < 4u; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < 4u; ++j) {
            if (pads[i].valid() && pads[j].valid() && pads[i] == pads[j]) {
                return false;
            }
        }
    }
    return true;
}

struct SpiConfig {
    SpiRole role = SpiRole::host;
    SpiMode mode = SpiMode::mode0;
    SpiClock clock = SpiClock::div16;
    SpiDataSize bits = SpiDataSize::bits8;
    bool lsb_first = false;
    SpiNss nss = SpiNss::software;
    SpiDirection direction = SpiDirection::full_duplex;
    bool crc = false;
    uint16_t crc_polynomial = 0x0007u;
    bool dma_transmit = false;
    bool dma_receive = false;
};

/// The refusals the chapter owes: CRC "can only be used in full-duplex
/// mode" (16.3.1), a polynomial of zero computes nothing, a client has
/// no NSS output (SSOE is a host verb), and a receive-only client with
/// a hardware select is fine but a half-duplex host that also declares
/// CRC is not.
constexpr bool spi_config_valid(const SpiConfig& c) {
    if (c.crc && (c.direction != SpiDirection::full_duplex || c.crc_polynomial == 0u)) {
        return false;
    }
    if (c.role == SpiRole::client && c.nss == SpiNss::hardware_output) {
        return false;
    }
    return true;
}

/// What CTLR1 holds for a configuration, SPE clear.
constexpr uint16_t spi_ctlr1_of(const SpiConfig& c) {
    uint16_t v = 0;
    if (spi_mode_cpha(c.mode)) { v |= spi_cpha; }
    if (spi_mode_cpol(c.mode)) { v |= spi_cpol; }
    if (c.role == SpiRole::host) { v |= spi_mstr; }
    v |= static_cast<uint16_t>(static_cast<uint16_t>(c.clock) << 3);
    if (c.lsb_first) { v |= spi_lsbfirst; }
    if (c.nss == SpiNss::software) {
        v |= spi_ssm;
        if (c.role == SpiRole::host) { v |= spi_ssi; }   // a client under SSM keeps SSI low
    }
    if (c.bits == SpiDataSize::bits16) { v |= spi_dff; }
    if (c.crc) { v |= spi_crcen; }
    switch (c.direction) {
        case SpiDirection::full_duplex:     break;
        case SpiDirection::receive_only:    v |= spi_rxonly; break;
        case SpiDirection::half_duplex_out: v |= spi_bidimode | spi_bidioe; break;
        case SpiDirection::half_duplex_in:  v |= spi_bidimode; break;
    }
    return v;
}

/// What CTLR2 holds for a configuration, the interrupt enables clear.
constexpr uint16_t spi_ctlr2_of(const SpiConfig& c) {
    uint16_t v = 0;
    if (c.nss == SpiNss::hardware_output) { v |= spi_ssoe; }
    if (c.dma_transmit) { v |= spi_txdmaen; }
    if (c.dma_receive) { v |= spi_rxdmaen; }
    return v;
}

// =============================================================================
// The resource
// =============================================================================

/**
 * Spi<n>: the register block and its verbs. `configure()` REFUSES what
 * spi_config_valid() refuses and writes the two control words with SPE
 * clear; `enable()` raises SPE; `disable()` runs the drain procedure.
 * The ISR body returns the raised-and-enabled sources for the caller to
 * act on; it consumes nothing.
 */
template <uint8_t n>
struct Spi {
    static_assert(spi_base_for(n) != 0, "brio Spi: this family has SPI1 alone");

    Spi() = delete;

    static constexpr uint8_t number = n;
    /// Table 8-2: the two channels this instance's requests reach.
    static constexpr uint8_t dma_rx_channel = 2;
    static constexpr uint8_t dma_tx_channel = 3;

    static SpiRegs& regs() { return *reinterpret_cast<SpiRegs*>(spi_base_for(n)); }
    static constexpr Irq irq() { return Irq::spi1; }
    static volatile void* data_address() { return &regs().DATAR; }

    static void bus_clock(bool on) {
        if (on) { rcc()->PB2PCENR |= rcc_pb2_spi1; } else { rcc()->PB2PCENR &= ~rcc_pb2_spi1; }
    }
    /// The RCC reset pulse: every register back to its reset value.
    static void reset() {
        rcc()->PB2PRSTR |= rcc_pb2_spi1;
        rcc()->PB2PRSTR &= ~rcc_pb2_spi1;
    }

    /// The whole configuration, with SPE clear on the way in (DFF and
    /// CRCEN are enable-protected, 16.3.1). Refused by the chapter's
    /// rules; the interrupt enables are kept as they are.
    static bool configure(const SpiConfig& c) {
        if (!spi_config_valid(c)) {
            return false;
        }
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        if (c.crc) {
            regs().CRCR = c.crc_polynomial;
        }
        regs().CTLR1 = spi_ctlr1_of(c);
        regs().CTLR2 = static_cast<uint16_t>((regs().CTLR2 & (spi_txeie | spi_rxneie | spi_errie)) |
                                             spi_ctlr2_of(c));
        return true;
    }

    static void enable() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_spe); }
    static bool enabled() { return (regs().CTLR1 & spi_spe) != 0u; }

    /// The drain-then-disable procedure: the last frame back (RXNE),
    /// the buffer empty (TXE), the shifter idle (not BSY), then SPE
    /// down. Each wait is bounded; false says one ran out and the bit
    /// was dropped regardless.
    static bool disable() {
        bool ok = true;
        if (enabled()) {
            ok = wait_until([] { return txe(); }) && wait_until([] { return !busy(); });
        }
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        return ok;
    }

    // ---- data -------------------------------------------------------------

    /// Write one frame; a 16-bit store lands the low byte in 8-bit mode.
    static void data(SpiDataSize, uint16_t v) { regs().DATAR = v; }
    /// Read one frame, which is also what clears RXNE.
    static uint16_t data(SpiDataSize bits) {
        return static_cast<uint16_t>(regs().DATAR & spi_frame_mask(bits));
    }
    static void data8(uint8_t v) { regs().DATAR = v; }
    static uint8_t data8() { return static_cast<uint8_t>(regs().DATAR); }

    static bool rxne() { return (regs().STATR & spi_rxne) != 0u; }
    static bool txe() { return (regs().STATR & spi_txe) != 0u; }
    static bool busy() { return (regs().STATR & spi_bsy) != 0u; }
    static uint16_t status() { return regs().STATR; }

    /// Throw away whatever the receive buffer holds and the OVR it may
    /// have raised.
    static void flush_rx() {
        (void)regs().DATAR;
        (void)regs().STATR;
    }

    // ---- errors (16.2.7) --------------------------------------------------

    static bool overrun() { return (regs().STATR & spi_ovr) != 0u; }
    /// DATAR then STATR, in that order.
    static void clear_overrun() {
        (void)regs().DATAR;
        (void)regs().STATR;
    }
    static bool mode_fault() { return (regs().STATR & spi_modf) != 0u; }
    /// STATR then a CTLR1 write, in that order. MODF has also cleared
    /// SPE and MSTR; the caller reconfigures.
    static void clear_mode_fault() {
        (void)regs().STATR;
        regs().CTLR1 = regs().CTLR1;
    }
    static bool crc_error() { return (regs().STATR & spi_crcerr) != 0u; }
    static void clear_crc_error() { regs().STATR = static_cast<uint16_t>(~spi_crcerr); }

    // ---- CRC (16.2.5) -----------------------------------------------------

    /// Send the transmit CRC after the frame just written.
    static void crc_next() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_crcnext); }
    static uint16_t rx_crc() { return regs().RCRCR; }
    static uint16_t tx_crc() { return regs().TCRCR; }

    // ---- the chip select and the rest ------------------------------------

    /// SSI, for a configuration under SSM: high is "not selected" on a
    /// host (and what keeps it a host), low selects a client.
    static void software_select(bool selected) {
        if (selected) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_ssi);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_ssi);
        }
    }

    /// HSCR.HSRXEN: the high-speed read mode, whose SCK the manual gives
    /// as HCLK/(BR + 2). Not measured (the file header).
    static void high_speed_read(bool on) {
        if (on) { regs().HSCR = static_cast<uint16_t>(regs().HSCR | spi_hsrxen); }
        else { regs().HSCR = static_cast<uint16_t>(regs().HSCR & ~spi_hsrxen); }
    }

    static void dma_requests(bool tx, bool rx) {
        uint16_t v = static_cast<uint16_t>(regs().CTLR2 & ~(spi_txdmaen | spi_rxdmaen));
        if (tx) { v |= spi_txdmaen; }
        if (rx) { v |= spi_rxdmaen; }
        regs().CTLR2 = v;
    }

    // ---- interrupts -------------------------------------------------------

    static void rxne_interrupt(bool on) { interrupt(spi_rxneie, on); }
    static void txe_interrupt(bool on) { interrupt(spi_txeie, on); }
    static void error_interrupt(bool on) { interrupt(spi_errie, on); }

    /// The raised sources that are ENABLED, the way the other strata's
    /// ISR bodies answer: RXNE and TXE under their own enables, the
    /// three errors under ERRIE. Nothing is cleared here.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint16_t st = regs().STATR;
        const uint16_t en = regs().CTLR2;
        uint32_t up = 0;
        if ((en & spi_rxneie) != 0u && (st & spi_rxne) != 0u) { up |= SpiFlag::rxne; }
        if ((en & spi_txeie) != 0u && (st & spi_txe) != 0u) { up |= SpiFlag::txe; }
        if ((en & spi_errie) != 0u) {
            up |= st & (spi_crcerr | spi_modf | spi_ovr);
        }
        return up;
    }

private:
    static void interrupt(uint16_t bit, bool on) {
        if (on) { regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 | bit); }
        else { regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 & ~bit); }
    }

    /// A bounded wait: true when the condition came, false when the
    /// budget ran out (a bus whose clock never runs must not hang the
    /// kernel; the budget is generous against the slowest frame).
    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }
};

// =============================================================================
// The host engine
// =============================================================================

/// A DMA block the engines could not finish (a transfer error on either
/// channel, or a polled block that never completed): the engine's own
/// status code, in the range util/bus_master.hpp leaves to engines.
inline constexpr uint8_t spi_dma_fault = bus_engine_status;

/**
 * SpiHost<n, pins, TxEngine, RxEngine>
 *
 * The engine SpiBus (util/spi_bus.hpp = BusMaster) drives. Its Request
 * is the other strata's: the bus AO asserts the request's own chip
 * select around the transaction, a COMMAND phase goes out with D/C low
 * and a DATA phase with it high, the data phase has an optional out
 * buffer (null = 0xFF dummies) and an optional in buffer (null = the
 * frames read back are discarded), and every buffer is LENT until the
 * reply lands. The mode, rate and frame size travel per request, so a
 * shared bus serves devices that disagree on them; the bit order is a
 * property of the WIRE and a bus-level verb.
 *
 * THE PUMP RUNS ON RXNE. A frame written to DATAR is clocked out and
 * the frame that comes back raises RXNE when it has been shifted BOTH
 * ways - which is the one moment the bus is provably idle, and why
 * TXE (raised when the buffer moves to the shifter, a frame early) is
 * never the pump's edge. Per frame: one write, one interrupt, one read.
 *
 * TWO COMPLETION STYLES, the request's choice: `polled` false runs the
 * data on this interrupt with the kernel free between frames (a
 * TransferDone follows), `polled` true spins inside start() with only
 * this SPI's own RXNE silenced and completes synchronously - at HCLK/2
 * a frame is sixteen cycles and an interrupt entry alone costs more,
 * so the polled loop is the fast path for bulk.
 *
 * THE ENGINE SLOTS: `DmaTxEngine<3>` and `DmaRxEngine<2>`, both or
 * neither (the data phase is full duplex and the transaction's
 * completion is the RECEIVE block's), on the two channels table 8-2
 * wires to this instance and no others. A request in 16-bit frames
 * falls back to the pump: the engines carry bytes.
 *
 * FRAMES IN A BYTE BUFFER: one byte per 8-bit frame, two bytes
 * low-first per 16-bit frame - the other strata's rule.
 */
template <uint8_t n, SpiPins pins = spi1_default_pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class SpiHost {
    using S = Spi<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from ch32v00x/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is "
                  "full-duplex and its completion is the RECEIVE block's");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the two engines must ride two different DMA channels");
    static_assert([] {
        if constexpr (TxEngine::present) {
            return TxEngine::channel == S::dma_tx_channel && RxEngine::channel == S::dma_rx_channel;
        } else {
            return true;
        }
    }(), "brio SpiHost: on this family the channel IS the request (RM table 8-2) - SPI1 "
         "transmits on DMA channel 3 and receives on channel 2, and an engine on any "
         "other channel would move nothing");
    static_assert(spi_pins_valid(pins),
                  "brio SpiHost: these SPI pads are not a link - SCK is required and no "
                  "two signals may name the same pad");
    static_assert(pins.mosi.valid(),
                  "brio SpiHost: a host needs SCK and MOSI; MISO is optional only for a "
                  "write-only bus, and the pump reads whatever comes back either way");

    using SckPin = Pin<pins.sck.port, pins.sck.pin>;
    using MosiPin = Pin<pins.mosi.port, pins.mosi.pin>;
    using MisoPin = Pin<pins.miso.valid() ? pins.miso.port : pins.sck.port,
                        pins.miso.valid() ? pins.miso.pin : pins.sck.pin>;
    using NssPin = Pin<pins.nss.valid() ? pins.nss.port : pins.sck.port,
                       pins.nss.valid() ? pins.nss.pin : pins.sck.pin>;

public:
    SpiHost() = delete;

    using Resource = S;
    static constexpr SpiPins pin_pads = pins;
    static constexpr bool has_engines = TxEngine::present;

    struct Request {
        PinRef cs;   ///< asserted low around the transaction
        PinRef dc;   ///< display D/C line; null = no such pin
        /// Microseconds between the CS assertion and the first clock -
        /// what a device's datasheet calls CS setup. Spent spinning in
        /// start(), main context; 0 = none.
        uint8_t cs_setup_us = 0;
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        uint8_t cmd_len;   ///< in FRAMES (see the class comment)
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len;      ///< phase 2 length, in FRAMES
        ReplyTo<SpiDone> reply;

        /// Per-transaction bus configuration: a shared bus's devices
        /// each name their own, and the engine reprograms the peripheral
        /// only when something CHANGED.
        SpiClock clock = SpiClock::div16;
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;

        /// Completion style: false = per-frame ISR pump, true = POLLED
        /// inside start() (see the class comment).
        bool polled = false;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the instance up as a host: gate, reset, configuration,
    /// pads, the PFIC line. `clock` is the app's Clock tag, the one
    /// truth of HCLK (this family has no APB prescaler, so HCLK is what
    /// BR divides). `max_sck_hz` is an optional CEILING for the whole
    /// bus, re-resolved on rebase(); 0 = none. False when the ceiling
    /// cannot be produced at this clock, or the boot configuration is
    /// refused.
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its SCK ceiling and its cs_setup timing "
                      "would go stale on a clock change");
        Pfic::disable(S::irq());
        ceiling_hz_ = max_sck_hz;
        rebase(clock_hz(clock));
        if (max_sck_hz != 0u && !ceiling_) {
            return false;   // not even HCLK/256 honours it
        }
        S::bus_clock(true);
        S::reset();
        Afio::remap_spi1(pins.remap);
        applied_ = boot_config();
        if (!S::configure(applied_)) {
            return false;
        }
        status_ = spi_ok;
        if constexpr (has_engines) {
            TxEngine::arm(S::data_address());
            RxEngine::arm(S::data_address());
        }
        // The pads go to the peripheral only now, with the registers
        // already holding the idle polarity: a pad handed over first
        // would park undriven on the bus, and a client watching SCK
        // cannot tell a glitch from an edge. SPE is raised last.
        SckPin::function();
        MosiPin::function();
        if constexpr (pins.miso.valid()) {
            MisoPin::input();
        }
        // THE NSS PAD IS NOT CLAIMED: the boot configuration is
        // SpiNss::software and 16.2.1 makes the pad free under it - which
        // is what this engine's chip select is, an ordinary GPIO the
        // Request carries. claim_nss_pad() is for a program that moves
        // the resource to a hardware arrangement.
        S::enable();
        S::flush_rx();
        Pfic::enable(S::irq());
        return true;
    }

    /// HCLK changed (DynamicClock fan-out). A Request's `clock` is a
    /// DIVISION of HCLK and scales by itself; what is recomputed is the
    /// ceiling and the cs_setup timing. THE BUS MUST BE IDLE.
    static void rebase(uint32_t hz) {
        hclk_hz_ = hz;
        ceiling_ = ceiling_hz_ != 0u ? spi_rate_for(hz, ceiling_hz_) : std::optional<SpiClock>{};
        cs_rate_ = delay_rate(hz);
    }

    /// The BR code that produces at most `hz` of SCK at the HCLK last
    /// seen - the chooser a device's datasheet limit is spoken to.
    static std::optional<SpiClock> clock_for(uint32_t hz) { return spi_rate_for(hclk_hz_, hz); }
    /// What a request at this code really runs at, ceiling included.
    static uint32_t sck_hz(SpiClock c) { return spi_sck_hz(hclk_hz_, clamp(c)); }
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    static uint32_t reference_hz() { return hclk_hz_; }

    /// Put the peripheral at this mode, rate and frame size NOW, moving
    /// no data - for callers that frame the select window themselves
    /// (Request.cs null). A mode change is a CPOL FLIP ON THE WIRE and a
    /// selected client counts it as an edge: prime FIRST, then select.
    static void prime(SpiMode m, SpiClock c, SpiDataSize bits = SpiDataSize::bits8) {
        apply(m, clamp(c), bits);
    }

    /// The bus's bit order (LSBFIRST): a property of the WIRE, applied
    /// once, not a per-request field. The bus must be idle.
    static bool bit_order(bool lsb_first) {
        if (applied_.lsb_first == lsb_first) {
            return true;
        }
        applied_.lsb_first = lsb_first;
        (void)S::disable();
        const bool ok = S::configure(applied_);
        S::enable();
        return ok;
    }
    static bool lsb_first() { return applied_.lsb_first; }

    // ---- the transfer -------------------------------------------------------

    /// Begin a transaction (SpiBus calls it from main context). True when
    /// it completed SYNCHRONOUSLY (polled requests, the empty one); false
    /// when it runs on the ISR and a TransferDone follows - exactly
    /// util/bus_master.hpp's engine contract.
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
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
                        S::rxne_interrupt(true);
                        S::data(req_.bits, frame_at(req_.cmd.get(), 0));
                        return false;
                    }
                    launch_dma();
                    return false;   // dma_isr() is the completion edge
                }
                S::rxne_interrupt(false);
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
            S::rxne_interrupt(true);
            S::data(req_.bits, first_frame());   // the ISR pumps the rest
            return false;
        }
        // Polled pump: silence this SPI's own RXNE (a bound handler would
        // steal the frames). Global interrupts STAY ENABLED.
        S::rxne_interrupt(false);
        for (uint8_t i = 0; i < r.cmd_len; ++i) {
            (void)xfer(frame_at(r.cmd.get(), i));
        }
        r.dc.set();
        for (uint16_t i = 0; i < r.len; ++i) {
            const uint16_t in = xfer(data_frame(i));
            if (r.rx.get() != nullptr) {
                store_frame(r.rx.get(), i, in);
            }
        }
        r.cs.set();
        return true;
    }

    /// The instance's interrupt body - call from its vector. Only RXNE
    /// is ever armed by this engine, and reading DATAR is both the
    /// capture and the acknowledgement. True when the transaction just
    /// completed (CS released): the edge the app's glue posts
    /// TransferDone on.
    [[gnu::always_inline]] static bool isr() {
        if ((S::isr() & SpiFlag::rxne) == 0u) {
            return false;
        }
        const uint16_t in = S::data(req_.bits);

        if constexpr (has_engines) {
            if (dma_serves(req_)) {
                // Only the COMMAND phase runs on this pump when the
                // engines serve the request; its echo is discarded.
                (void)in;
                ++pos_;
                if (pos_ >= req_.cmd_len) {
                    in_cmd_ = false;
                    S::rxne_interrupt(false);
                    req_.dc.set();
                    if (req_.len == 0u) {
                        req_.cs.set();
                        return true;
                    }
                    launch_dma();
                    return false;
                }
                S::data(req_.bits, frame_at(req_.cmd.get(), pos_));
                return false;
            }
        }

        if (!in_cmd_ && req_.rx.get() != nullptr) {
            store_frame(req_.rx.get(), pos_, in);
        }
        ++pos_;
        if (in_cmd_ && pos_ >= req_.cmd_len) {
            in_cmd_ = false;
            pos_ = 0;
            req_.dc.set();
        }
        if (!in_cmd_ && pos_ >= req_.len) {
            S::rxne_interrupt(false);
            req_.cs.set();
            return true;
        }
        S::data(req_.bits, next_frame());
        return false;
    }

    /// The DMA channels' interrupt body - call from BOTH channels'
    /// vectors (one vector per channel here). A transfer error on either
    /// channel ends the transaction with spi_dma_fault. Compiles away on
    /// an engineless host. True when the transaction just completed.
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
    /// released FIRST, the engines put away and re-claimed, the
    /// peripheral reset and reconfigured to the applied state. False
    /// when a bounded wait ran out.
    static bool recover() {
        req_.cs.set();
        if constexpr (has_engines) {
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address());
        }
        Pfic::disable(S::irq());
        in_cmd_ = false;
        dma_active_ = false;
        dma_done_ = false;
        S::rxne_interrupt(false);
        const bool ok = S::disable();
        S::reset();
        const bool cfg = S::configure(applied_);
        S::enable();
        S::flush_rx();
        Pfic::enable(S::irq());
        return ok && cfg;
    }

    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Pfic::disable(S::irq());
        (void)S::disable();
        S::bus_clock(false);
        SckPin::release();
        MosiPin::release();
        if constexpr (pins.miso.valid()) {
            MisoPin::release();
        }
    }

    /// Hand the NSS pad to the peripheral - only for a program that has
    /// moved the resource to a hardware arrangement. The task's own
    /// transactions never use it.
    static void claim_nss_pad(bool on) {
        if constexpr (pins.nss.valid()) {
            if (on) { NssPin::function(); } else { NssPin::release(); }
        } else {
            (void)on;
        }
    }

private:
    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio SpiHost: the DMA engines must carry uint8_t elements - the Request's "
         "buffers are bytes and the engined path serves 8-bit frames");

    /// The engines serve a request in 8-bit frames; 16-bit ones are two
    /// bytes a datum, which byte engines cannot express.
    static bool dma_serves(const Request& r) { return !spi_frame_is_halfword(r.bits); }

    static void launch_dma() {
        dma_done_ = false;
        dma_active_ = true;
        // The RECEIVE channel first: its request rises only when a frame
        // has come back, and the transmit side is what starts the clock.
        if (req_.rx.get() != nullptr) {
            (void)RxEngine::start(req_.rx.get(), req_.len);
        } else {
            (void)RxEngine::start_discard(&rx_sink_, req_.len);
        }
        if (req_.tx.get() != nullptr) {
            (void)TxEngine::start(req_.tx.get(), req_.len);
        } else {
            (void)TxEngine::start_fixed(&tx_dummy_, req_.len);
        }
    }

    /// One exit for the data phase. True when the ISR-style caller
    /// should post completion.
    static bool finish_dma(uint8_t st) {
        if (st != spi_ok) {
            status_ = st;
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address());
        } else if (TxEngine::busy()) {
            // The receive block completing is proof the transmit one did:
            // every frame that came back was clocked out first.
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
    /// slowest frame is 256 x 16 HCLK cycles; the budget scales).
    static void spin_dma() {
        uint32_t spins = 200'000u + 6'000u * static_cast<uint32_t>(req_.len);
        while (!dma_done_ && spins-- != 0u) {
        }
        if (!dma_done_) {
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address());
            dma_active_ = false;
            status_ = spi_dma_fault;
        }
    }

    /// A slower rate is a LARGER code, so the ceiling clamps from below.
    static SpiClock clamp(SpiClock c) {
        if (!ceiling_) {
            return c;
        }
        return static_cast<uint8_t>(c) < static_cast<uint8_t>(*ceiling_) ? *ceiling_ : c;
    }

    static SpiConfig boot_config() {
        SpiConfig c{};
        c.role = SpiRole::host;
        c.mode = SpiMode::mode0;
        c.bits = SpiDataSize::bits8;
        c.clock = ceiling_ ? *ceiling_ : SpiClock::div16;
        c.nss = SpiNss::software;
        if constexpr (has_engines) {
            c.dma_transmit = true;
            c.dma_receive = true;
        }
        return c;
    }

    /// Put the peripheral where this request wants it. DFF is
    /// enable-protected and BR/CPOL/CPHA "cannot be modified during
    /// communication", so any change costs a disable/enable pair -
    /// paid only when something moved.
    static void apply(SpiMode m, SpiClock c, SpiDataSize bits) {
        if (m == applied_.mode && c == applied_.clock && bits == applied_.bits) {
            return;
        }
        applied_.mode = m;
        applied_.clock = c;
        applied_.bits = bits;
        (void)S::disable();
        (void)S::configure(applied_);
        S::enable();
    }

    /// One polled frame: write, spin on RXNE, read back. Bounded.
    static uint16_t xfer(uint16_t out) {
        S::data(req_.bits, out);
        uint32_t spins = 200'000u;
        while (!S::rxne() && spins-- != 0u) {
        }
        return S::data(req_.bits);
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
    static uint16_t first_frame() { return in_cmd_ ? frame_at(req_.cmd.get(), 0) : data_frame(0); }
    static uint16_t next_frame() { return in_cmd_ ? frame_at(req_.cmd.get(), pos_) : data_frame(pos_); }
    static uint16_t data_frame(uint16_t i) {
        return (req_.tx.get() != nullptr) ? frame_at(req_.tx.get(), i) : 0xFFFFu;
    }

    static inline Request req_{};
    static inline uint16_t pos_ = 0;
    static inline bool in_cmd_ = false;
    static inline uint8_t status_ = spi_ok;
    static inline volatile bool dma_done_ = false;
    static inline volatile bool dma_active_ = false;
    static inline uint8_t rx_sink_ = 0;
    static constexpr uint8_t tx_dummy_ = 0xFF;
    static inline SpiConfig applied_{};
    static inline uint32_t hclk_hz_ = 0;
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
 * The other end of the wire: SCK, MOSI and NSS are inputs, the host
 * sets the pace, and the only thing this side controls is WHAT it has
 * ready to shift out when the next clock arrives.
 *
 * ONE FRAME AHEAD. With no FIFO the transmit buffer holds exactly the
 * next answer: 16.2.3 says the first frame moves from the buffer into
 * the shifter on the first sampling edge, and TXE rises then - the
 * moment to write the NEXT one. So: load the first answer before the
 * host selects (`enable(first)`), then write frame k + 1 on TXE while
 * frame k shifts; a write made on RXNE instead reaches the buffer one
 * frame late under a host that clocks back to back. A client that
 * falls behind does not stall the bus: what the host reads is whatever
 * the shifter held (16.2.3 makes no underrun flag), a wrong answer and
 * never a missing one.
 *
 * THE NSS PAD IS THE TRANSACTION: read directly (selected()), since the
 * peripheral publishes no select status. `drive_output` makes this
 * client a DARK LISTENER: MISO is handed to the peripheral only for the
 * window it answers in, so a shared harness stays uncontested.
 */
template <uint8_t n, SpiPins pins = spi1_default_pins>
class SpiClient {
    using S = Spi<n>;

    static_assert(spi_pins_valid(pins),
                  "brio SpiClient: these SPI pads are not a link - SCK is required and no "
                  "two signals may name the same pad");
    static_assert(pins.mosi.valid(), "brio SpiClient: a client listens on MOSI");

    using SckPin = Pin<pins.sck.port, pins.sck.pin>;
    using MosiPin = Pin<pins.mosi.port, pins.mosi.pin>;
    using MisoPin = Pin<pins.miso.valid() ? pins.miso.port : pins.sck.port,
                        pins.miso.valid() ? pins.miso.pin : pins.sck.pin>;
    using NssPin = Pin<pins.nss.valid() ? pins.nss.port : pins.sck.port,
                       pins.nss.valid() ? pins.nss.pin : pins.sck.pin>;

public:
    SpiClient() = delete;

    using Resource = S;
    static constexpr SpiPins pin_pads = pins;
    static constexpr bool has_nss_pad = pins.nss.valid();

    struct Config {
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;
        bool lsb_first = false;
        /// software: the pad is free and select() drives SSI.
        /// hardware_input: the NSS pad IS the chip select (the default).
        SpiNss nss = SpiNss::hardware_input;
        SpiDirection direction = SpiDirection::full_duplex;
        bool crc = false;
        uint16_t crc_polynomial = 0x0007u;
        /// Whether the MISO pad is handed to the peripheral at all.
        bool drive_output = true;
    };

    /// Bring the instance up as a client. The bus clock is required
    /// even though the shifter runs on the host's SCK. False when the
    /// configuration is refused.
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        Pfic::disable(S::irq());
        S::bus_clock(true);
        S::reset();
        Afio::remap_spi1(pins.remap);
        if (!S::configure(config_of(cfg))) {
            return false;
        }
        // Inputs first, the output last.
        SckPin::input();
        MosiPin::input();
        if constexpr (has_nss_pad) {
            NssPin::input(PinPull::up);
        }
        drive_output(cfg.drive_output);
        S::flush_rx();
        Pfic::enable(S::irq());
        return true;
    }

    /// Hand the answer line to the peripheral, or take it back (a
    /// floating input, driving nothing).
    static void drive_output(bool on) {
        if constexpr (pins.miso.valid()) {
            if (on) { MisoPin::function(); } else { MisoPin::release(); }
        } else {
            (void)on;
        }
    }

    /// SPE up and the FIRST ANSWER in the buffer before the host's clock
    /// can arrive. Call it with NSS still high.
    static void enable(uint16_t first = 0xFFFFu) {
        S::enable();
        write(first);
    }
    static bool disable() { return S::disable(); }

    /// How many answers must be queued ahead of the frame the host is
    /// about to clock: ONE here - no FIFO, one buffer.
    static constexpr uint8_t frames_ahead = 1;

    /// Load the next frame to shift out; on TXE, for the one-ahead rule.
    static void write(uint16_t v) { S::data(bits_, v); }
    static bool writable() { return S::txe(); }

    /// One received frame, or nothing. Reading DATAR clears RXNE.
    static std::optional<uint16_t> poll() {
        if (!S::rxne()) {
            return {};
        }
        return S::data(bits_);
    }

    /// Is the host holding NSS low right now? A live pad read. Always
    /// false without an NSS pad.
    static bool selected() {
        if constexpr (has_nss_pad) {
            return !NssPin::read();
        } else {
            return false;
        }
    }

    /// The software select, for a client under SSM.
    static void select(bool on) { S::software_select(on); }

    static bool overrun() { return S::overrun(); }
    static void clear_overrun() { S::clear_overrun(); }
    static bool crc_error() { return S::crc_error(); }
    static void clear_crc_error() { S::clear_crc_error(); }
    static uint16_t status() { return S::status(); }

    /// The ISR body: the raised-and-enabled sources, for the app's glue.
    [[gnu::always_inline]] static uint32_t isr() { return S::isr(); }

    static void rxne_interrupt(bool on) { S::rxne_interrupt(on); }
    static void txe_interrupt(bool on) { S::txe_interrupt(on); }
    static void error_interrupt(bool on) { S::error_interrupt(on); }

    static SpiDataSize bits() { return bits_; }

    static void release() {
        Pfic::disable(S::irq());
        (void)S::disable();
        S::bus_clock(false);
        SckPin::release();
        MosiPin::release();
        if constexpr (pins.miso.valid()) {
            MisoPin::release();
        }
        if constexpr (has_nss_pad) {
            NssPin::release();
        }
    }

private:
    static SpiConfig config_of(const Config& c) {
        bits_ = c.bits;
        SpiConfig s{};
        s.role = SpiRole::client;
        s.mode = c.mode;
        s.bits = c.bits;
        s.lsb_first = c.lsb_first;
        s.nss = c.nss;
        s.direction = c.direction;
        s.crc = c.crc;
        s.crc_polynomial = c.crc_polynomial;
        return s;
    }

    static inline SpiDataSize bits_ = SpiDataSize::bits8;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// 16.3.1's BR table at the 48 MHz this stratum's PLL clock produces.
static_assert(spi_sck_hz(48'000'000UL, SpiClock::div2) == 24'000'000UL);
static_assert(spi_sck_hz(48'000'000UL, SpiClock::div256) == 187'500UL);
static_assert(spi_rate_for(48'000'000UL, 24'000'000UL) == SpiClock::div2);
static_assert(spi_rate_for(48'000'000UL, 23'999'999UL) == SpiClock::div4);
static_assert(spi_rate_for(48'000'000UL, 1'000'000UL) == SpiClock::div64);
static_assert(spi_rate_for(48'000'000UL, 187'500UL) == SpiClock::div256);
// A ceiling below HCLK/256 is REFUSED, never rounded up.
static_assert(!spi_rate_for(48'000'000UL, 187'499UL).has_value());
static_assert(!spi_rate_for(0, 1'000'000UL).has_value());

static_assert(!spi_frame_is_halfword(SpiDataSize::bits8) && spi_frame_is_halfword(SpiDataSize::bits16));
static_assert(spi_frame_mask(SpiDataSize::bits8) == 0xFFu);

// Table 16-1.
static_assert(!spi_mode_cpol(SpiMode::mode1) && spi_mode_cpha(SpiMode::mode1));
static_assert(spi_mode_cpol(SpiMode::mode2) && !spi_mode_cpha(SpiMode::mode2));

// The refusals spi_config_valid() owes the chapter.
static_assert(spi_config_valid(SpiConfig{}));
static_assert(!spi_config_valid(SpiConfig{.direction = SpiDirection::half_duplex_out, .crc = true}));
static_assert(!spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0}));
static_assert(!spi_config_valid(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));
static_assert(spi_pins_valid(spi1_default_pins) && spi_pins_valid(spi1_pins_for(4)));
static_assert(spi1_pins_for(2).sck == Pad{'D', 2} && spi1_pins_for(2).remap == 2u);
static_assert(!spi_pins_valid(SpiPins{.sck = {'C', 5}, .mosi = {'C', 5}}));
static_assert(!spi_pins_valid(SpiPins{.mosi = {'C', 6}}));
static_assert(!spi_pins_valid(SpiPins{.sck = {'C', 5}, .mosi = {'C', 6}, .remap = 7}));

// The control words: mode 3 host at /16 under software select.
static_assert(spi_ctlr1_of(SpiConfig{.mode = SpiMode::mode3, .clock = SpiClock::div16}) ==
              (spi_cpha | spi_cpol | spi_mstr | (3u << 3) | spi_ssm | spi_ssi));
static_assert(spi_ctlr2_of(SpiConfig{.nss = SpiNss::hardware_output, .dma_receive = true}) ==
              (spi_ssoe | spi_rxdmaen));

} // namespace brio
