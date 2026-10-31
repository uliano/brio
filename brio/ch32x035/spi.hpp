/*
 * spi.hpp
 *
 * The SPI of the CH32X035 (RM V1.8 ch. 16) as a bus for a program: a
 * display, a flash chip or a converter driven from an active object
 * through the arbiter of util/spi_bus.hpp, or this chip as a client
 * another host clocks. One instance, both roles, 8- or 16-bit frames,
 * the four clock modes, a software or hardware chip select, hardware
 * CRC, the simplex arrangements, two DMA requests - the STM32F1's SPI
 * under WCH's register names with two registers' worth of its own: HSCR
 * (16.3.8, a "high-speed read" mode) and CTLR2.ODEN (16.3.2, open-drain
 * NSS, SCK and MOSI). Three layers, the shape every bus driver of brio
 * has:
 *
 *  - `Spi<n>` is the resource: the register block, its gate and reset,
 *    the configuration under the rules the chapter states, the data and
 *    status verbs, the error clearing sequences, the CRC, the interrupt
 *    body that reads the raised-and-enabled sources. It decides nothing.
 *  - `SpiHost<n, pins, TxEngine, RxEngine, hold_off_cycles, Placement>`
 *    is the engine util/spi_bus.hpp's SpiBus drives: the Request is the
 *    same struct on every stratum, field for field (a chip select, a D/C
 *    line, a command phase, a data phase with optional out and in, the
 *    per-request mode, rate and frame size, the polled or asynchronous
 *    way to run it), so an application that fills it by name runs here
 *    unchanged.
 *  - `SpiClient<n, pins>` is the other end of the wire: the polled runs a
 *    client's transaction is served by (answer, exchange, receive) and
 *    the per-frame verbs that are their degenerate case.
 *
 * The smallest use, a command and a block of pixels to a display, is
 * written out at SpiHost below: `using Bus = brio::SpiHost<1, pins,
 * DmaTxEngine<3, uint8_t>, DmaRxEngine<2, uint8_t>>` on the column the
 * QFN20 bonds with the instance's two DMA channels, `SpiBus<Bus, P, 4>`
 * as the arbiter in the program's Tenuto, `Bus::init(clock, 8'000'000)`
 * after the clock task with a ceiling for the bus, the two channels'
 * vectors bound to dma_isr() with BRIO_CH32_VECTOR, then a Request
 * posted with its chip select and its buffers lent until the reply.
 *
 * Who calls what, and from where. init(), release(), bit_order() and
 * claim_nss_pad() are setup verbs, main context; init() waits for
 * nothing. start() is called by the arbiter from its dispatch, main
 * context, and a request runs one of two ways (docs/design/spi-bus.md,
 * "Two ways to run a request"). Polled, it completes inside start(), the
 * caller spinning for the whole transaction, the chip-select setup time
 * included - bounded by a poll budget, so a dead bus ends spi_stalled
 * and not a hang. Asynchronous, it rides the DMA engines: start() arms
 * them and returns, and dma_isr() ends it; a request too short to repay
 * the engines' fixed cost runs polled instead and completes inside
 * start() (async_min_frames() below); and a host with no engines bound
 * refuses it, spi_rejected at once, nothing on the wire. dma_isr() is
 * the two channels' vectors' body: the program's handlers call it and
 * nothing else does; it runs to completion and masks nothing. The host
 * arms no interrupt of the SPI block: the instance's vector is not its.
 * recover() is the arbiter's, on a tenure that never answered, and
 * waits, bounded, for the block to drain. The Request's three buffers
 * are lent from the post to the SpiDone the arbiter replies with:
 * untouched and in scope until then, which nothing checks. The
 * client's runs are polled, main-context verbs that spin for a whole
 * transaction under a spin budget; its isr() is its vector's body.
 * Nothing here masks interrupts.
 *
 * What this silicon offers that bears on cost, each item used or declined
 * (docs/ch32x035/spi.md holds the inventory with the measurements):
 *
 *  - NO FIFO: one transmit buffer, one shift register, one receive buffer
 *    (figure 16-1). TXE rises "when the data has reached the shift
 *    register" (16.2.2) - a frame written then follows the one shifting
 *    with no gap, "if the TXE flag position bit needs to fill the data
 *    register with data to maintain the complete data flow" - and RXNE at
 *    "the last sample clock edge of the data word". The receive buffer is
 *    ONE frame deep: a frame that completes while the previous one stands
 *    unread sets OVR (16.2.7) and is lost. Used: the transmit-only loop
 *    writes on TXE, the receive loop writes ahead as far as the one-deep
 *    receive buffer allows under the image's hold-off.
 *  - 8 or 16 bits (DFF), and the beat of an engine is the frame: DATAR is
 *    sixteen bits. Used.
 *  - SCK = HCLK / 2^(BR+1), /2 to /256 (16.3.1), "up to half of FHCLK"
 *    (16.1), 24 MHz at 48 - the datasheet's table 3-19 ceiling for both
 *    roles. Every bus of this series runs at HCLK, so the rate's
 *    reference is HCLK. Used.
 *  - HSCR.HSRXEN, "read enable in SPI high speed mode (CLK greater than or
 *    equal to 36MHz)", "only valid when the clock is divided by 2" and
 *    "not readable" (16.3.8) - where 16.3.1's note on BR says that with
 *    the bit set "the SCK frequency is FHCLK/(BR+2)". MEASURED: 16.3.1 is
 *    right - with the bit set every code gives HCLK/(BR+2), a second
 *    ladder /2, /3 .. /9 - and a read of HSCR answers zero whatever was
 *    written. The two texts agree at BR = 000 alone, so the host sets the
 *    bit only while the code in force is /2, keeps a copy, and drops it
 *    before any other code is written: a powers-of-two code means the same
 *    rate with the mode asked or not. At /2 a client's answer has no
 *    timing margin without it: table 3-19 gives a client's data valid up
 *    to 25 ns after its shifting edge and the host 5 ns of setup, 30 ns
 *    against the 20.8 ns half period of 24 MHz. MEASURED against the
 *    CH32V006's client in mode 3: without the bit, a 32-frame window on
 *    the engines read one frame's first bit as 1 where the client sent 0
 *    in 29 of 70 windows; with it, none in 28. So
 *    `SpiHost::Config::high_speed_read` is ON by default (applied at /2
 *    alone); the second ladder is declined (docs/ch32x035/spi.md).
 *  - CTLR2.ODEN, open-drain NSS, SCK and MOSI "to support low-voltage SPI
 *    signals" (16.3.2): the one pad class this GPIO has for an SPI - the
 *    port has no speed field and no open-drain output of its own (RM
 *    8.3.1.1) - so it is `SpiHost::Config::open_drain`, written once
 *    (measured: SCK never rises on a wire with no pull-up, NSS returns
 *    through the far end's). It covers the host's three outputs and NOT a
 *    client's MISO, so a client has no pad class at all.
 *  - CRC (16.2.5): two calculators, CRCNEXT after the last frame, CRCERR.
 *    The resource's verbs; a tenure shape with a CRC is born with a
 *    device that checks one.
 *  - BIDIMODE/BIDIOE and RXONLY (16.2.4): the resource's configuration;
 *    the engine is full duplex (a receive-only host clocks for as long as
 *    SPE stands, which no Request can bound), the client's dark listener
 *    is a pad left undriven rather than RXONLY.
 *  - The hardware select (16.2.1): SSOE drives NSS low "when the SPI host
 *    sends output" - measured: low from the host's first frame (not at
 *    SPE), held low between frames, high again only when SPE clears - and
 *    a low NSS on a host without SSOE raises MODF and demotes it (MSTR and
 *    SPE cleared, measured with a second chip's pulse). An SSOE select
 *    spans every transaction from the first frame to the next disable, so
 *    it frames no transaction of the Request's: the engine's select is a
 *    GPIO the Request carries (docs/design/spi-bus.md), and
 *    `claim_nss_pad()` hands the pad over for a program that moves the
 *    resource to a hardware arrangement.
 *  - DMA requests on channels 2 (receive) and 3 (transmit) (RM 9.2.3,
 *    ch32x035/dma_engine.hpp's table): the channel IS the request, so an
 *    engine slot on any other channel is refused at compile time. Used:
 *    the asynchronous way, one interrupt a phase (the receive block's)
 *    and at the wire at every code to /2, where this core's own loops are
 *    not; and inside start() for a long polled read.
 *  - The instruction fetch: the code runs from a flash with two wait
 *    states at 48 MHz behind a prefetch (RM 1.2, 20.3.1), measured at three
 *    to four cycles an instruction in these loops, a taken branch the
 *    dearest - and costing what its address modulo four makes it, so the
 *    polled loops' functions are aligned to the fetch's word - and SRAM
 *    as the faster place. Offered, the program's choice,
 *    for the paths that run once a FRAME: the host's last template
 *    parameter is a placement (util/placement.hpp), `InFlash` by default,
 *    and under `InRam` the two polled loops run from SRAM
 *    (`.fast_text.brio`, copied with .data by the crt) - the
 *    transmit-only loop at the wire at /2. The cost constants follow the
 *    placement, each measured both ways. The paths that run once a
 *    transaction (start(), the engines' launch) stay in the flash under
 *    either (docs/ch32x035/spi.md, "The host above the wire").
 *  - Interrupts: TXE, RXNE and the three errors under ERRIE on ONE vector
 *    (table 7-1, entry 33). Declined by the host: a per-frame interrupt
 *    costs more than the frame at every rate this core can serve one at
 *    (225 to 268 cycles of handler against 32 a byte at /4, measured), so
 *    the asynchronous way is the engines' and the vector is the
 *    client's (SpiClient::isr()).
 *
 * The chapter's two mode numberings disagree, and this file uses neither.
 * Table 16-1 prints CPOL 0/1/1/1 and CPHA 0/0/0/1 for modes 0..3 - a
 * table no numbering produces - and figures 16-2..16-9 number the modes
 * by CPHA first (mode 1 = CPHA 0, CPOL 1; mode 2 = CPHA 1, CPOL 0). The
 * `SpiMode` here is the Motorola numbering every other stratum speaks,
 * mode = CPOL x 2 + CPHA, written into the bits by name.
 *
 * The client's two restrictions are measured, not read. 16.2.3: "If
 * transmitting in slave mode is required, CPOL needs to be set and
 * configured as mode 2 or mode 3"; 16.3.1: LSBFIRST - "LSB is only
 * supported by SPI as host". Measured as a client under a foreign host
 * (test_x035_spi letters r and w): this die receives exact in all four
 * modes and both bit orders, answers exact in all four modes MSb first -
 * the first restriction does not hold here - and LSb first its MISO never
 * moves, in every mode - the second holds, for what a client sends. So
 * SpiClient refuses an answering client LSb first and nothing else
 * (spi_client_answers below).
 *
 * The disable procedure: clearing SPE while a frame shifts cuts it on the
 * wire. `Spi::disable()` waits TXE and then not BSY, each bounded, before
 * it drops the bit. MODF is cleared by a STATR access followed by a CTLR1
 * write, OVR by a DATAR read followed by a STATR read (16.2.7) - two
 * sequences the resource keeps.
 *
 * The pads are a column (afio.hpp, SPI1_RM[1:0]): four columns, of which
 * the QFN20 bonds the first alone (PA4 NSS, PA5 SCK, PA6 MISO, PA7 MOSI).
 * spi1_pins_for(code) is a whole column and init() writes the code. A pad
 * that shares its package pin with another (the datasheet's notes 4 to 7:
 * PA7 with PB0 on the CH32X033F8P6) may be an input and never an output,
 * so a host whose MOSI sits on one does not compile, and a client whose
 * MISO does not either.
 */

#pragma once

#include <stdint.h>

#include <cstddef>
#include <optional>
#include <span>
#include <type_traits>

#include "ch32x035/afio.hpp"
#include "ch32x035/clock.hpp"
#include "ch32x035/delay.hpp"
#include "ch32x035/device.hpp"
#include "ch32x035/dma_engine.hpp"
#include "ch32x035/pfic.hpp"
#include "ch32x035/pin.hpp"
#include "ch32x035/platform.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/placement.hpp"
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
    uint32_t RESERVED7[2];     ///< 0x1c, 0x20 (no register in table 16-2)
    volatile uint16_t HSCR;    ///< 0x24 high-speed control, WRITE-ONLY (16.3.8)
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
inline constexpr uint16_t spi_oden     = 1u << 15;
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

/// The four clock modes in the MOTOROLA numbering, mode = CPOL x 2 + CPHA
/// - not table 16-1's, and not figures 16-2..16-9's (the file header).
enum class SpiMode : uint8_t { mode0 = 0, mode1 = 1, mode2 = 2, mode3 = 3 };

constexpr bool spi_mode_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 2u) != 0u; }
constexpr bool spi_mode_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 1u) != 0u; }

/// BR[2:0]: HCLK over two to the code plus one (16.3.1, HSRXEN clear).
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

/// DFF: the two frame sizes.
enum class SpiDataSize : uint8_t { bits8 = 0, bits16 = 1 };

constexpr bool spi_frame_is_halfword(SpiDataSize s) { return s == SpiDataSize::bits16; }
constexpr uint8_t spi_data_bits(SpiDataSize s) { return s == SpiDataSize::bits16 ? 16u : 8u; }
constexpr uint16_t spi_frame_mask(SpiDataSize s) { return s == SpiDataSize::bits16 ? 0xFFFFu : 0x00FFu; }

enum class SpiRole : uint8_t { client = 0, host = 1 };

/// The chip select arrangements of 16.2.1: `software` is SSM/SSI (the pad
/// free); `hardware_output` is SSOE, the pad driven low while a host
/// transmits; `hardware_input` is the pad as the select - a client's chip
/// select, and on a host the multi-host input whose low level raises
/// MODF and demotes it.
enum class SpiNss : uint8_t { software, hardware_output, hardware_input };

/// 16.2.4's line arrangements.
enum class SpiDirection : uint8_t {
    full_duplex,       ///< two data lines, both ways
    receive_only,      ///< RXONLY: two lines, the output one released
    half_duplex_out,   ///< BIDIMODE + BIDIOE: one line, transmitting
    half_duplex_in,    ///< BIDIMODE, BIDIOE clear: one line, receiving
};

/// Which pads carry the four signals, and the remap code that puts the
/// peripheral on them (afio.hpp, SPI1_RM[1:0]): init() writes the code. A
/// signal the program does not wire is an invalid Pad.
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

/// SPI1's default column: SCK PA5, MOSI PA7, MISO PA6, NSS PA4 - the one
/// column the QFN20 bonds.
inline constexpr SpiPins spi1_default_pins = spi1_pins_for(0);

/// Is `p` the column's own pad for that signal, or no pad at all?
constexpr bool spi_pad_of_column(Pad p, Pad column) { return !p.valid() || p == column; }

/// A link: a remap code the field has, SCK named and bonded, every named
/// pad bonded by this package and the column's own (the remap moves a
/// whole column: a pad from another one would carry nothing), no two
/// signals on one pad.
constexpr bool spi_pins_valid(const SpiPins& p) {
    if (p.remap >= afio_spi1_codes || !p.sck.valid()) {
        return false;
    }
    const SpiPadSet col = afio_spi1_pads(p.remap);
    if (!spi_pad_of_column(p.sck, col.sck) || !spi_pad_of_column(p.mosi, col.mosi) ||
        !spi_pad_of_column(p.miso, col.miso) || !spi_pad_of_column(p.nss, col.nss)) {
        return false;
    }
    const Pad pads[] = {p.sck, p.mosi, p.miso, p.nss};
    for (uint8_t i = 0; i < 4u; ++i) {
        if (pads[i].valid() && !pad_bonded(pads[i])) {
            return false;
        }
    }
    return true;
}

/// A whole configuration of the block.
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
    /// CTLR2.ODEN: NSS, SCK and MOSI open drain (16.3.2).
    bool open_drain = false;
    /// HSCR.HSRXEN (16.3.8): a host's high-speed read, /2 only.
    bool high_speed_read = false;
};

/// The refusals the chapter owes: CRC "can only be used in full-duplex
/// mode" (16.3.1), a polynomial of zero computes nothing, a client has no
/// NSS output (SSOE is a host verb), and the high-speed read is a host's
/// and "only valid when the clock is divided by 2" (16.3.8).
constexpr bool spi_config_valid(const SpiConfig& c) {
    if (c.crc && (c.direction != SpiDirection::full_duplex || c.crc_polynomial == 0u)) {
        return false;
    }
    if (c.role == SpiRole::client && c.nss == SpiNss::hardware_output) {
        return false;
    }
    if (c.high_speed_read && (c.role != SpiRole::host || c.clock != SpiClock::div2)) {
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
        if (c.role == SpiRole::host) { v |= spi_ssi; }   // a client under SSM keeps SSI low (16.2.3)
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
    if (c.open_drain) { v |= spi_oden; }
    return v;
}

// =============================================================================
// The resource
// =============================================================================

/**
 * Spi<n>: the register block and its verbs. `configure()` refuses what
 * spi_config_valid() refuses and writes the control words with SPE clear,
 * HSCR whole (its one bit is write-only, so the resource keeps a copy);
 * `enable()` raises SPE; `disable()` runs the drain procedure. The ISR
 * body returns the raised-and-enabled sources; it consumes nothing.
 */
template <uint8_t n>
struct Spi {
    static_assert(spi_base_for(n) != 0, "brio Spi: this series has SPI1 alone (RM ch. 16)");

    Spi() = delete;

    static constexpr uint8_t number = n;
    /// RM 9.2.3 (ch32x035/dma_engine.hpp's table): the two channels this
    /// instance's requests reach.
    static constexpr uint8_t dma_rx_channel = dma_request_channel(DmaRequest::spi1_rx);
    static constexpr uint8_t dma_tx_channel = dma_request_channel(DmaRequest::spi1_tx);
    static_assert(dma_rx_channel == 2u && dma_tx_channel == 3u,
                  "RM 9.2.3: SPI_RX on channel 2, SPI_TX on channel 3");

    static SpiRegs& regs() { return *reinterpret_cast<SpiRegs*>(spi_base_for(n)); }
    static constexpr Irq irq() { return Irq::spi1; }
    static volatile void* data_address() { return &regs().DATAR; }

    static void bus_clock(bool on) {
        if (on) { Rcc::enable(Bus::pb2, rcc_pb2_spi1); } else { Rcc::disable(Bus::pb2, rcc_pb2_spi1); }
    }
    static bool bus_clock() { return Rcc::enabled(Bus::pb2, rcc_pb2_spi1); }
    /// The RCC reset pulse: every register back to its reset value, HSCR
    /// included - and with it the copy.
    static void reset() {
        (void)Rcc::reset(Bus::pb2, rcc_pb2_spi1);
        hscr_ = 0;
    }

    /// The whole configuration, with SPE clear on the way in (DFF and
    /// CRCEN "can only be written when SPE is 0", 16.3.1). Refused by the
    /// chapter's rules; the interrupt enables are kept as they are.
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
        high_speed_read_raw(c.high_speed_read);
        return true;
    }

    static void enable() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_spe); }
    static bool enabled() { return (regs().CTLR1 & spi_spe) != 0u; }

    /// The drain-then-disable procedure: the buffer empty (TXE), the
    /// shifter idle (not BSY), then SPE down. Each wait is bounded; false
    /// says one ran out and the bit was dropped regardless.
    static bool disable() {
        bool ok = true;
        if (enabled()) {
            ok = wait_until([] { return txe(); }) && wait_until([] { return !busy(); });
        }
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        return ok;
    }

    // ---- data -------------------------------------------------------------

    /// Write one frame; in 8-bit mode the low byte is the frame.
    static void data(uint16_t v) { regs().DATAR = v; }
    /// Read one frame, which is also what clears RXNE. In 8-bit mode the
    /// high byte reads zero (16.3.4).
    static uint16_t data() { return regs().DATAR; }

    static bool rxne() { return (regs().STATR & spi_rxne) != 0u; }
    static bool txe() { return (regs().STATR & spi_txe) != 0u; }
    static bool busy() { return (regs().STATR & spi_bsy) != 0u; }
    static uint16_t status() { return regs().STATR; }

    /// Throw away what the receive buffer holds and the OVR it may have
    /// raised: 16.2.7's sequence, DATAR then STATR.
    static void flush_rx() {
        (void)regs().DATAR;
        (void)regs().STATR;
    }

    // ---- errors (16.2.7) --------------------------------------------------

    static bool overrun() { return (regs().STATR & spi_ovr) != 0u; }
    /// DATAR then STATR, in that order.
    static void clear_overrun() { flush_rx(); }
    static bool mode_fault() { return (regs().STATR & spi_modf) != 0u; }
    /// "First perform a read or write operation to R16_SPI1_STATR, and then
    /// write R16_SPI1_CTLR1." MODF has also cleared SPE and MSTR; the
    /// caller reconfigures.
    static void clear_mode_fault() {
        (void)regs().STATR;
        regs().CTLR1 = regs().CTLR1;
    }
    static bool crc_error() { return (regs().STATR & spi_crcerr) != 0u; }
    /// CRCERR is RW0 (16.3.3): a zero written clears it, a one keeps.
    static void clear_crc_error() { regs().STATR = static_cast<uint16_t>(~spi_crcerr); }

    // ---- CRC (16.2.5) -----------------------------------------------------

    /// Send the transmit CRC after the frame just written: "set
    /// immediately after the last data is written to the data register".
    static void crc_next() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_crcnext); }
    /// Read with BSY clear (16.3.6, 16.3.7).
    static uint16_t rx_crc() { return regs().RCRCR; }
    static uint16_t tx_crc() { return regs().TCRCR; }

    // ---- the chip select, the pads' class, the high-speed read ------------

    /// SSI, for a configuration under SSM: high is "not selected" on a
    /// host (and what keeps it a host), low selects a client.
    static void software_select(bool selected) {
        if (selected) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_ssi);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_ssi);
        }
    }

    /// CTLR2.ODEN: NSS, SCK and MOSI open drain (16.3.2).
    static void open_drain(bool on) {
        if (on) { regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 | spi_oden); }
        else { regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 & ~spi_oden); }
    }
    static bool open_drain() { return (regs().CTLR2 & spi_oden) != 0u; }

    /// HSCR.HSRXEN, refused (false, nothing written) where 16.3.8 says the
    /// mode is not valid: on a client, and at any code but /2 - read from
    /// CTLR1 as it stands. The register is written whole (its one bit is
    /// write-only, nothing to read-modify-write) and the copy kept.
    static bool high_speed_read(bool on) {
        const uint16_t c1 = regs().CTLR1;
        if (on && ((c1 & spi_mstr) == 0u || (c1 & spi_br_mask) != 0u)) {
            return false;
        }
        high_speed_read_raw(on);
        return true;
    }
    /// The copy of HSRXEN: the bit itself is not readable (16.3.8).
    static bool high_speed_read() { return (hscr_ & spi_hsrxen) != 0u; }
    /// HSCR as a read answers - what the "not readable" means on this die
    /// is a measurement (test_x035_spi letter a), never a decision.
    static uint16_t hscr_register() { return regs().HSCR; }
    /// The store itself, unchecked: for the suite that measures what
    /// 16.3.1 and 16.3.8 disagree on, and for the host engine, which
    /// calls it only at /2.
    static void high_speed_read_raw(bool on) {
        hscr_ = on ? spi_hsrxen : uint16_t{0};
        regs().HSCR = hscr_;
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

    /// The raised sources that are enabled: RXNE and TXE under their own
    /// enables, the three errors under ERRIE (16.2.8). Nothing is cleared.
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

    /// A bounded wait: a bus whose clock never runs must not hang the
    /// kernel; the budget is generous against the slowest frame.
    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = 100'000u; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }

    /// HSCR as last written (the bit is write-only).
    static inline uint16_t hscr_ = 0;
};

// =============================================================================
// The host engine
// =============================================================================

/// A DMA block the engines could not finish (a transfer error on either
/// channel, or a polled block that never completed): the engine's own
/// status code, in the range util/bus_master.hpp leaves to engines.
inline constexpr uint8_t spi_dma_fault = bus_engine_status;
// The two other codes this host answers are the vocabulary's
// (util/spi_bus.hpp): spi_overrun, a frame lost to the one-deep receive
// buffer (16.2.7) because a handler read it later than one frame time
// after it came back - every frame went out, one that came back is
// missing; spi_stalled, a polled flag that never came within the
// transaction's budget - a clock that stopped, a block that is dead.

/// A frame's time on the wire in HCLK cycles: its bits at SCK = HCLK over
/// two to the code plus one (16.3.1's BR table, HSRXEN clear).
constexpr uint32_t spi_frame_cycles(SpiClock c, SpiDataSize bits) {
    return static_cast<uint32_t>(spi_data_bits(bits)) << (static_cast<uint8_t>(c) + 1u);
}

/*
 * The write-ahead threshold, and the arithmetic behind it.
 *
 * A frame written while another shifts waits in the transmit buffer and
 * moves into the shifter the instant the one before it ends (16.2.2), so
 * keeping two frames in flight (frame k + 1 shifting while frame k's
 * answer is read) keeps the bus busy through the reader's latency; one in
 * flight leaves the bus idle from RXNE to the next write. Two in flight
 * has a price: the receive buffer is one frame deep. RXNE for frame k
 * rises at its last sampling edge, frame k + 1 ends one frame time later,
 * and a frame k not read by then is lost under OVR. The polled receive
 * loop, in main context, takes frame k within
 *
 *     the longest window an interrupt holds the loop off
 *   + the core's interrupt entry
 *   + the loop's own path back to the DATAR read
 *
 * cycles after RXNE, and two in flight is safe only where a frame lasts
 * longer than that sum. No interrupt nests on this core (kernel.md section
 * 1, "The model in one page", the platform's promise), so the first term is the longer of the
 * image's longest handler and its longest masked window: the hold-off, a
 * fact of the image and not of the family, which is why it is the host's
 * last template parameter. Its default is the longest this family's own
 * images measure (docs/ch32x035/spi.md, "The host above the wire"). The
 * second and third terms are the hardware prologue's entry (37 cycles,
 * measured, docs/ch32x035/platform.md) and the loop's return to its read,
 * counted in the release listing.
 */
/// The default of SpiHost's `hold_off_cycles`: the one safe with every
/// driver in the flash, the default placement (util/placement.hpp) - the
/// longest handler this family's images measure so, the I2C host's event
/// pump, 333 cycles of body at most (test_x035_i2c letter x, its stamps
/// and its owner switch included, docs/ch32x035/i2c.md), and the 36 of its
/// exit, rounded up. The USART's engined completion measures 293 at most
/// from the flash (docs/ch32x035/usart.md), the console's handler 151, the
/// tick's 67 (test_x035_spi letter b, stamps included). The hold-off is a
/// fact of the image, not of one driver: a program that places its
/// handlers in SRAM shortens them, and states its own hold-off as this
/// host's template parameter. The write-ahead codes are the same for any
/// hold-off from 199 to 454 cycles (the static_asserts at the end of this
/// file).
inline constexpr uint32_t spi_default_hold_off_cycles = 380;
/// The hardware prologue's entry (37 cycles, measured) and the receive
/// loop's way back to its DATAR read when the handler returns, about 20
/// cycles: the exit's last instructions and the loop's poll turn, fetched
/// from the flash.
inline constexpr uint32_t spi_read_latency_cycles = 37u + 20u;

/// The frame, in HCLK cycles, a read can come after RXNE at the latest
/// under a hold-off: a longer one keeps two in flight.
constexpr uint32_t spi_write_ahead_min_frame_cycles(uint32_t hold_off_cycles) {
    return hold_off_cycles + spi_read_latency_cycles;
}

/// Two frames in flight at this code and width, under this hold-off?
constexpr bool spi_write_ahead_safe(SpiClock c, SpiDataSize bits, uint32_t hold_off_cycles) {
    return spi_frame_cycles(c, bits) > spi_write_ahead_min_frame_cycles(hold_off_cycles);
}
/// The fastest code at which two frames are in flight for a width under a
/// hold-off (every slower code is too).
constexpr SpiClock spi_write_ahead_from(SpiDataSize bits, uint32_t hold_off_cycles) {
    for (uint8_t code = 0; code < 8u; ++code) {
        if (spi_write_ahead_safe(static_cast<SpiClock>(code), bits, hold_off_cycles)) {
            return static_cast<SpiClock>(code);
        }
    }
    return SpiClock::div256;
}

/*
 * THE SHORT ASYNCHRONOUS REQUEST, and the arithmetic behind it.
 *
 * On the engines a transaction costs the core their fixed cost, E (start(),
 * the launch, the one interrupt, the completion), whatever its length: the
 * wire's time is the core's own. Polled, it costs the core the whole
 * transaction: a fixed part F and each frame's time, which is the wire's
 * frame or the loop's own turn, the longer - for a frame of W HCLK cycles
 * at the code and width (spi_frame_cycles()),
 *
 *     a frame read (an in buffer)   max(W + the receive loop's dead bus, its shortest turn)
 *     a frame written (none)        max(W, the transmit-only loop's shortest turn)
 *
 * So a request is cheaper polled while F plus its frames' cycles stays
 * under E, and rides the engines from there: spi_async_min_frames() gives
 * the smallest count N with F + N x frame >= E, ceil((E - F) / frame). Only
 * the difference E - F enters, one for a read and one for a write (the
 * transmit-only loop's fixed part is not the receive loop's), and it is
 * measured as one: the engines' 32-frame block less its 32 frames and a
 * polled line's fixed part, both timed through the same call of the suite,
 * so that the caller's own cycles - which move by hundreds with where its
 * code sits in the flash - cancel. The figures are a placement's
 * (SpiHostCosts below), measured by test_x035_spi letter b on its engined
 * host, which also finds the edge itself; the counts they give are pinned
 * by the static_asserts at the end of this file. The host folds a request's cost
 * from two frame figures it keeps for the code in force - a frame read and
 * a frame written, computed when the code changes and not per request - as
 * cmd_len frames written and len frames read or written: two multiplies
 * and a compare in start(), on the asynchronous path alone. The one case
 * the rule approximates: a command phase on the engines is a block of its
 * own, judged by the same E, though a block chained from the interrupt
 * costs a launch and an interrupt, less than E.
 */
/// What the two ways cost the core on this series, for one placement of
/// the host's per-frame paths (the host's cost constants are these).
struct SpiHostCosts {
    uint32_t engines_over_read;      ///< E - F for a read, at /4 (the receive loop's F)
    uint32_t engines_over_written;   ///< E - F for a write, at /4 (the transmit-only loop's F)
    uint32_t polled_gap;             ///< the receive loop's dead bus a frame, one in flight, at /4
    uint32_t read_turn;              ///< the receive loop's shortest frame, at /2
    uint32_t write_turn;             ///< the transmit-only loop's shortest frame, at /2
};
/// Measured on the CH32X035F8U6 at 48 MHz in one session for each
/// placement (test_x035_spi letter b, its engined hosts). E - F runs start()
/// from the flash under either placement; the frames are the placement's,
/// and in SRAM they move with where the image puts the loops
/// (docs/ch32x035/spi.md, "The host above the wire").
inline constexpr SpiHostCosts spi_costs_in_flash{660u, 705u, 20u, 52u, 24u};
inline constexpr SpiHostCosts spi_costs_in_ram{590u, 580u, 10u, 27u, 16u};

/// What a polled frame costs the core at a code and width, read or
/// written (the arithmetic above).
constexpr uint32_t spi_polled_frame_cost(const SpiHostCosts& k, SpiClock c, SpiDataSize bits, bool read) {
    const uint32_t w = spi_frame_cycles(c, bits);
    if (read) {
        return w + k.polled_gap > k.read_turn ? w + k.polled_gap : k.read_turn;
    }
    return w > k.write_turn ? w : k.write_turn;
}
/// E - F: what the engines cost beyond a polled transaction's fixed part,
/// a read's (`read`) or a write's.
constexpr uint32_t spi_async_budget(const SpiHostCosts& k, bool read) {
    return read ? k.engines_over_read : k.engines_over_written;
}
/// The frames from which an asynchronous request of one phase rides the
/// engines at a code and width (`read`: with an in buffer); a shorter one
/// runs polled and completes inside start().
constexpr uint16_t spi_async_min_frames(const SpiHostCosts& k, SpiClock c, SpiDataSize bits, bool read) {
    const uint32_t f = spi_polled_frame_cost(k, c, bits, read);
    const uint32_t n = (spi_async_budget(k, read) + f - 1u) / f;
    return static_cast<uint16_t>(n > 1u ? n : 1u);
}

/**
 * SpiHost<n, pins, TxEngine, RxEngine, hold_off_cycles, Placement>
 *
 * The engine SpiBus (util/spi_bus.hpp = BusMaster) drives. The bus
 * active object asserts the request's own chip select around the
 * transaction, a command phase goes out with D/C low and a data phase
 * with it high, the data phase has an optional out buffer (null = 0xFF
 * dummies) and an optional in buffer (null = the frames read back are
 * discarded), and every buffer
 * is lent until the reply lands. The mode, rate and frame size travel per
 * request; the bit order and the pads' class are the wire's.
 *
 *   using Bus = brio::SpiHost<1, brio::spi1_default_pins,     // PA5 SCK, PA7 MOSI, PA6 MISO
 *                             brio::DmaTxEngine<3, uint8_t>, brio::DmaRxEngine<2, uint8_t>>;
 *   using Spi = brio::SpiBus<Bus, P, 4>;          // the arbiter, in the Tenuto
 *   using Cs = brio::Pin<'A', 4>;
 *   Bus::init(clock, 8'000'000);                  // after the clock task; a ceiling for the bus
 *   Cs::output(true);
 *
 *   Bus::Request r{};
 *   r.cs = Cs::ref();
 *   r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
 *   r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
 *   r.clock = brio::SpiClock::div8;               // 6 MHz at 48
 *   r.mode = brio::SpiMode::mode0;
 *   r.reply = brio::reply_to<Display, brio::SpiDone>();
 *   brio::post<Spi>(r);                           // asynchronous: polled stays false
 *
 *   BRIO_CH32_VECTOR(dma1_channel2_handler) {
 *       if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
 *   }
 *   BRIO_CH32_VECTOR(dma1_channel3_handler) {
 *       if (Bus::dma_isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
 *   }
 *
 * The Request is lent for the call (util/bus_master.hpp): a request that
 * completes inside start() is read through the reference; one that goes
 * to the engines has what its tenure needs - the two pins, the three
 * buffers, the two lengths, the completion style - copied as word stores
 * into the host's tenure record. The Request's fields are laid out so that
 * those are its first eight words and nothing in it is padding.
 *
 * The two ways (docs/design/spi-bus.md, "Two ways to run a request"):
 *
 *  - POLLED (`polled` true): the loops below inside start(), and a phase
 *    with an in buffer of `dma_min_frames_polled` frames or more on the
 *    engines inside start() too, start() waiting on their completion.
 *  - ASYNCHRONOUS (`polled` false) IS DMA. With no engines bound the
 *    request is refused: start() answers true at once with spi_rejected
 *    in status(), before it touches the block, the select or the D/C, so
 *    nothing reaches the wire; `has_dma` says so at compile time. With
 *    engines bound, a request whose polled transfer would cost less than
 *    the engines' fixed cost runs polled and completes inside start() -
 *    the rule, its arithmetic and its figures above SpiHostCosts. A
 *    longer one goes to the engines: a command phase on them too when it
 *    would be worth a block of its own (or is the whole request), polled
 *    inside start() otherwise, and the data phase on them, start()
 *    returning false and dma_isr() ending the transaction.
 *
 * The polled loops, from 16.2.2. A phase whose answers nobody wants (the
 * command phase, a write with no in buffer) runs the transmit-only loop: a
 * frame written whenever TXE says the buffer is free - a whole frame time
 * before the shifter needs it, so the clock never pauses and an interrupt
 * only pauses the stream - the answers left to overrun the receive buffer,
 * harmless to the transmit side (16.2.7), and cleared at the tail by the
 * DATAR-then-STATR read after TXE and then BSY say the last frame is out.
 * A phase with an in buffer runs the receive loop: one frame in flight
 * below the write-ahead threshold (write k, wait RXNE, read k and write
 * k + 1 in the next instruction), two above it (the phase primed with two,
 * frame k + 2 written as k is read). Both loops are specialized by frame
 * width and spend one bounded budget for the whole transaction.
 *
 * The engine slots: `DmaTxEngine<3, Elem>` and `DmaRxEngine<2, Elem>`,
 * both or neither (ch32x035/dma.hpp), on the two channels RM 9.2.3 wires
 * to this instance and no others. A block is full duplex and its
 * completion is the receive block's - every frame that came back was
 * clocked out first, so the transmit block has ended when the receive
 * block ends - and the transmit engine is armed for its errors alone: one
 * interrupt a block. The beat is the frame: engines of uint16_t move
 * a 16-bit request in half-word beats when its buffers sit on a
 * half-word boundary (RM 9.3.6: a wide access ignores the low address
 * bits), engines of uint8_t serve the 8-bit requests alone; a request
 * the engines cannot carry so - 16-bit frames from an odd address, or
 * over engines of uint8_t - runs polled inside start() whichever way it
 * asked, the one exception to "asynchronous is DMA", since the engines
 * cannot move it and a refusal would make the answer depend on where a
 * buffer happens to sit.
 *
 * The requests are raised around the engines and nowhere else: a request a
 * channel has not taken is held by this controller (measured on this part,
 * docs/ch32x035/dma.md), so RXDMAEN goes up only once the receive channel
 * is enabled, TXDMAEN only once the transmit one is, and both come down
 * at the block's end.
 *
 * Frames in a byte buffer: one byte per 8-bit frame, two bytes low-first
 * per 16-bit frame - one half-word in memory, which is what an engine's
 * beat moves.
 */
template <uint8_t n, SpiPins pins = spi1_default_pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine, uint32_t hold_off_cycles = spi_default_hold_off_cycles,
          typename Placement = InFlash>
class SpiHost {
    using S = Spi<n>;

    static_assert(HotPlacement<Placement, Ch32x035Platform<>>,
                  "brio SpiHost: the placement is util/placement.hpp's InFlash (the default) or InRam");

    static_assert(hold_off_cycles > 0u,
                  "brio SpiHost: a hold-off of zero cycles is no image's - an image with no "
                  "handler of its own still has the tick's, and every critical section is one");
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / DmaRxEngine "
                  "from ch32x035/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is "
                  "full-duplex and its completion is the RECEIVE block's");
    static_assert([] {
        if constexpr (TxEngine::present) {
            return sizeof(typename TxEngine::element) <= 2u && sizeof(typename RxEngine::element) <= 2u;
        } else {
            return true;
        }
    }(), "brio SpiHost: DATAR is sixteen bits - the engines' widest beat is uint8_t or uint16_t");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the two engines must ride two different DMA channels");
    static_assert(dma_engine_channel<TxEngine>() == 0u ||
                      (dma_engine_channel<TxEngine>() == S::dma_tx_channel &&
                       dma_engine_channel<RxEngine>() == S::dma_rx_channel),
                  "brio SpiHost: on this series the channel IS the request (RM 9.2.3) - SPI1 "
                  "transmits on DMA channel 3 and receives on channel 2, and an engine on any "
                  "other channel would wait for a request that never comes");
    static_assert(spi_pins_valid(pins),
                  "brio SpiHost: these SPI pads are not a link - SCK is required, every pad must "
                  "be its column's own (afio.hpp, SPI1_RM) and bonded by this part");
    static_assert(pins.mosi.valid(),
                  "brio SpiHost: a host needs SCK and MOSI; MISO is optional only for a "
                  "write-only bus");
    static_assert(pad_can_drive(pins.sck) && pad_can_drive(pins.mosi),
                  "brio SpiHost: the host DRIVES SCK and MOSI, and this part shares one of those "
                  "pads' package pin with another pad - the datasheet forbids it as an output "
                  "(table 2-1, notes 4 to 7; parts/<part>.hpp's port_twinned)");

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
    /// Can this host run an asynchronous request? The contract's name for
    /// has_engines (docs/design/spi-bus.md): false refuses every request
    /// not marked polled, so a program that posts one asserts it -
    /// `static_assert(Bus::has_dma)`.
    static constexpr bool has_dma = has_engines;

    /// The transaction descriptor. The field names are the contract (the
    /// same on every stratum, filled by name); the order is this host's: the
    /// eight words an asynchronous tenure copies first, then one word
    /// holding the setup time and the three fields apply() compares, then
    /// the reply - no padding anywhere.
    struct Request {
        PinRef cs;   ///< asserted low around the transaction
        PinRef dc;   ///< display D/C line; null = no such pin
        /// Phase 1, sent with DC low; lent until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        /// Phase 2 out, null = 0xFF dummies; lent until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; lent until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len;      ///< phase 2 length, in FRAMES
        uint8_t cmd_len;   ///< in FRAMES
        /// The way to run it: true = polled inside start(); false =
        /// asynchronous, on the DMA engines - polled inside start() when
        /// too short to repay them (async_min_frames()), refused
        /// (spi_rejected) on a host with none (has_dma).
        bool polled = false;
        /// Microseconds between the CS assertion and the first clock;
        /// spent spinning in start(), main context; 0 = none.
        uint8_t cs_setup_us = 0;
        /// Per-transaction bus configuration; the engine reprograms the
        /// block only when something changed.
        SpiClock clock = SpiClock::div16;
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;
        ReplyTo<SpiDone> reply;
    };
    static_assert(std::is_trivially_copyable_v<Request>);
    static_assert(sizeof(Request) == 40u, "the Request is forty bytes with no padding on this ABI");
    static_assert(offsetof(Request, cmd) == 16u && offsetof(Request, rx) == 24u && offsetof(Request, len) == 28u &&
                      offsetof(Request, polled) == 31u && offsetof(Request, cs_setup_us) == 32u &&
                      offsetof(Request, clock) == 33u && offsetof(Request, mode) == 34u &&
                      offsetof(Request, bits) == 35u && offsetof(Request, reply) == 36u,
                  "the tenure's words are the first eight of the Request, the configuration the ninth");

    /// What the bus is beyond the clock it runs from - written once by
    /// init(), never per transaction, because they are properties of the
    /// wire: the SCK ceiling; the pads' one class (ODEN: NSS, SCK and MOSI
    /// open drain, for a bus that carries its own pull-ups - this GPIO has
    /// no speed field, so this is the only choice the pads offer); and the
    /// high-speed read, HSRXEN, applied while the code in force is /2 and
    /// at no other (the file header).
    struct Config {
        uint32_t max_sck_hz = 0;   ///< the ceiling; 0 = none
        bool open_drain = false;
        bool high_speed_read = true;   ///< HSRXEN at /2: on, the file header
    };

    /// Is the host's per-frame path placed in SRAM (the file header)?
    static constexpr bool placed_in_ram = std::is_same_v<Placement, InRam>;

    /// The two ways' costs for this host's placement (SpiHostCosts above;
    /// test_x035_spi letter b judges each against its measurement).
    static constexpr SpiHostCosts costs = placed_in_ram ? spi_costs_in_ram : spi_costs_in_flash;
    /// E - F for a read and for a write (the arithmetic above SpiHostCosts).
    static constexpr uint32_t async_read_budget_cycles = spi_async_budget(costs, true);
    static constexpr uint32_t async_write_budget_cycles = spi_async_budget(costs, false);
    static constexpr uint32_t polled_gap_cycles = costs.polled_gap;
    static constexpr uint32_t polled_read_turn_cycles = costs.read_turn;
    static constexpr uint32_t polled_write_turn_cycles = costs.write_turn;
    /// A polled request with an in buffer rides the engines once the
    /// receive loop's dead bus would cost more than their extra fixed cost.
    /// The transmit-only loop never does: it runs within 5 % of the wire
    /// from /2.
    static constexpr uint16_t dma_min_frames_polled =
        static_cast<uint16_t>((async_read_budget_cycles + polled_gap_cycles - 1u) / polled_gap_cycles);
    static_assert(dma_min_frames_polled == (placed_in_ram ? 59u : 33u),
                  "the threshold the measured costs give (docs/ch32x035/spi.md)");
    /// The frames from which an asynchronous request of one phase rides the
    /// engines at this placement, a code and a width (`read`: with an in
    /// buffer); a shorter one runs polled and completes inside start().
    static constexpr uint16_t async_min_frames(SpiClock c, SpiDataSize bits, bool read) {
        return spi_async_min_frames(costs, c, bits, read);
    }

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the instance up as a host: gate, reset, the column, the
    /// configuration, the engines' binding, the pads. A setup verb, main(),
    /// after the clock task and before interrupts are enabled globally;
    /// waits for nothing. `clock` is the app's Clock tag, the one truth of
    /// HCLK - which BR divides, this series having no bus prescaler; the
    /// ceiling and the chip-select setup timing are computed from the rate
    /// its type claims. False when the ceiling cannot be produced at this
    /// clock or the boot configuration is refused. The instance's PFIC
    /// line is left disabled - the host arms no interrupt of the block -
    /// and the two channels' vectors are the program's to bind.
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not list it "
                      "among its Users: its SCK ceiling and its cs_setup timing would go stale "
                      "on a clock change");
        Pfic::disable(S::irq());
        ceiling_hz_ = cfg.max_sck_hz;
        open_drain_ = cfg.open_drain;
        high_speed_ = cfg.high_speed_read;
        rebase(clock_hz(clock));
        if (cfg.max_sck_hz != 0u && !ceiling_) {
            return false;   // not even HCLK/256 honours it
        }
        S::bus_clock(true);
        S::reset();
        Afio::remap(Remap::spi1, pins.remap);
        applied_ = boot_config();
        if (!S::configure(applied_)) {
            return false;
        }
        status_ = spi_ok;
        if constexpr (has_engines) {
            // The receive side at very_high, the transmit at high
            // (docs/design/dma.md): the receive half of DATAR holds one
            // frame, and a starved receive channel loses the next under OVR
            // (16.2.7); a starved transmit channel only stops the host's
            // clock between two frames. The transmit engine is armed for
            // its errors alone - the receive block's end is the transaction's.
            TxEngine::arm(S::data_address(), TxEngine::flag_error, DmaPriority::high);
            RxEngine::arm(S::data_address(), DmaPriority::very_high);
        }
        // The pads go to the peripheral only now, with the registers
        // already holding the idle polarity: a pad handed over first would
        // park undriven on the bus, and a client watching SCK cannot tell
        // a glitch from an edge. SPE is raised last.
        SckPin::function();
        MosiPin::function();
        if constexpr (pins.miso.valid()) {
            MisoPin::input();   // table 8-4: floating or pulled-up input
        }
        // The NSS pad is not claimed: the boot configuration is SSM/SSI and
        // 16.2.1 leaves the pad free under it - the engine's select is a
        // GPIO the Request carries. claim_nss_pad() is for a program that
        // moves the resource to a hardware arrangement.
        enable_applied();
        S::flush_rx();
        return true;
    }
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        return init(clock, Config{.max_sck_hz = max_sck_hz});
    }

    /// HCLK changed (DynamicClock fan-out). A Request's `clock` is a
    /// division of HCLK and scales by itself; the ceiling and the cs_setup
    /// timing are recomputed. The bus must be idle.
    static void rebase(uint32_t hz) {
        hclk_hz_ = hz;
        ceiling_ = ceiling_hz_ != 0u ? spi_rate_for(hz, ceiling_hz_) : std::optional<SpiClock>{};
        // A slower rate is a LARGER code, so the ceiling clamps codes from
        // below; code 0 as the floor clamps nothing ("no ceiling").
        floor_code_ = ceiling_ ? static_cast<uint8_t>(*ceiling_) : 0u;
        cs_rate_ = delay_rate(hz);
        // A new ceiling may clamp the last request's clock to another
        // code: the next request is folded and compared again.
        request_key_ = 0xFFFF'FFFFu;
    }

    static std::optional<SpiClock> clock_for(uint32_t hz) { return spi_rate_for(hclk_hz_, hz); }
    static uint32_t sck_hz(SpiClock c) { return spi_sck_hz(hclk_hz_, clamp(c)); }
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    static uint32_t reference_hz() { return hclk_hz_; }
    static bool open_drain() { return open_drain_; }
    /// Is the high-speed read asked for /2 (Config)? The bit itself is on
    /// only while /2 is in force: Spi::high_speed_read() is its copy.
    static bool high_speed_read() { return high_speed_; }

    /// The fastest code at which a receive of that width keeps two frames
    /// in flight under this host's hold-off.
    static constexpr SpiClock write_ahead_from(SpiDataSize bits) {
        return spi_write_ahead_from(bits, hold_off_cycles);
    }

    /// Put the peripheral at this mode, rate and frame size now, moving no
    /// data - for a caller that frames the select window itself (Request.cs
    /// null). A mode change is a CPOL flip on the wire and a selected
    /// client counts it as an edge: prime first, then select.
    static void prime(SpiMode m, SpiClock c, SpiDataSize bits = SpiDataSize::bits8) {
        Request r{};
        r.clock = c;
        r.mode = m;
        r.bits = bits;
        apply(r, clamp(c));
    }

    /// The bus's bit order (LSBFIRST): a property of the wire, applied
    /// once, not a per-request field. The bus must be idle.
    static bool bit_order(bool lsb_first) {
        if (applied_.lsb_first == lsb_first) {
            return true;
        }
        applied_.lsb_first = lsb_first;
        (void)S::disable();
        const bool ok = S::configure(applied_);
        enable_applied();
        return ok;
    }
    static bool lsb_first() { return applied_.lsb_first; }

    // ---- the transfer -------------------------------------------------------

    /// Begin a transaction (SpiBus calls it from main context). True when
    /// it completed inside the call - a polled request, an asynchronous one
    /// too short for the engines, the empty one, and on a host with no
    /// engines a refused one (spi_rejected, the wire untouched); false when
    /// it went to the engines and dma_isr() will end it, a TransferDone
    /// following - util/bus_master.hpp's engine contract.
    static bool start(const Request& r) {
        if constexpr (!has_dma) {
            if (!r.polled) {
                // Asynchronous is DMA, and this host has no engines:
                // refused before the block, the select or the D/C is
                // touched, so nothing reaches the wire.
                status_ = spi_rejected;
                return true;
            }
        }
        status_ = spi_ok;
        if (r.cmd_len == 0u && r.len == 0u) {
            return true;
        }
        const SpiClock code = clamp(r.clock);
        apply(r, code);
        if (r.cmd_len != 0u) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();   // assert, active low
        if (r.cs_setup_us != 0u) {
            (void)delay_us(cs_rate_, r.cs_setup_us);
        }
        // No flush of the receive buffer here: every path leaves it empty
        // at its end, and init() and recover() flush what a reset left.

        if constexpr (has_dma) {
            if (!r.polled && repays_engines(r)) {
                return start_engines(r, code);
            }
        }

        // Polled: asked, or an asynchronous request too short for the
        // engines (async_min_frames()).
        uint32_t budget = poll_budget(code, r.bits, static_cast<uint32_t>(r.cmd_len) + r.len);
        bool ok = true;
        if constexpr (has_dma) {
            if (r.rx.get() != nullptr && r.len >= dma_min_frames_polled && dma_serves(r)) {
                if (r.cmd_len != 0u) {
                    ok = poll_write_phase(r.bits, r.cmd.get(), r.cmd_len, budget);
                }
                r.dc.set();
                if (ok) {
                    keep(r);
                    tenure_.polled = true;   // spin_dma() waits; finish_dma() leaves the select
                    in_cmd_ = false;
                    step_ = spi_frame_is_halfword(r.bits) ? 2u : 1u;
                    launch_dma(r.tx.get(), r.rx.get(), r.len);
                    spin_dma();
                }
                r.cs.set();
                if (!ok) {
                    stalled();
                }
                return true;
            }
        }
        if (r.cmd_len != 0u) {
            ok = poll_write_phase(r.bits, r.cmd.get(), r.cmd_len, budget);
        }
        r.dc.set();
        if (ok && r.len != 0u) {
            if (r.rx.get() == nullptr) {
                ok = poll_write_phase(r.bits, r.tx.get(), r.len, budget);
            } else {
                ok = poll_duplex_phase(r.bits, r.tx.get(), r.rx.get(), r.len, budget,
                                       spi_write_ahead_safe(code, r.bits, hold_off_cycles));
            }
        }
        r.cs.set();
        if (!ok) {
            stalled();
        }
        return true;
    }

    /// The DMA channels' interrupt body - call from BOTH channels' vectors
    /// (one vector a channel here), and from nowhere else. The receive
    /// channel's completion ends a block: a command block's end raises D/C
    /// and puts the data block on the engines, a data block's end (or a
    /// command block with no data phase after it) ends the transaction,
    /// the select released. A transfer error on either channel ends it
    /// with spi_dma_fault. INTFR is read once for both. Compiles away on an
    /// engineless host. True when the transaction just completed: the edge
    /// the program's glue posts TransferDone{status()} on.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_dma) {
            const uint32_t intfr = RxEngine::block_flags();
            const uint8_t tx = TxEngine::service(intfr);
            if ((tx & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                return finish_dma(spi_dma_fault);
            }
            const uint8_t rx = RxEngine::service(intfr);
            if (rx != 0u && dma_active_) {
                if ((rx & RxEngine::flag_error) != 0u) {
                    return finish_dma(spi_dma_fault);
                }
                if ((rx & RxEngine::flag_complete) != 0u) {
                    if (in_cmd_) {
                        return next_block();
                    }
                    return finish_dma(spi_ok);
                }
            }
        }
        return false;
    }

    /// The engine's completion status, for the TransferDone payload: read
    /// in the vector right after dma_isr() answered true, or after a
    /// start() that answered true.
    static uint8_t status() { return status_; }

    /// Put the engine back where start() is legal (the verb a timed SpiBus
    /// calls on a transaction that never answered): the select released
    /// first, the engines put away, the block reset and reconfigured to
    /// the applied state. False when a bounded wait ran out.
    static bool recover() {
        tenure_.cs.set();
        if constexpr (has_dma) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();   // the binding stands: the next start() needs no arm()
        }
        in_cmd_ = false;
        dma_active_ = false;
        dma_done_ = false;
        const bool ok = S::disable();
        S::reset();
        const bool cfg = S::configure(applied_);
        enable_applied();
        S::flush_rx();
        return ok && cfg;
    }

    /// The block handed back: the engines stopped, the drain, the reset
    /// pulse before the gate closes (docs/design/dma.md's release
    /// contract: a raised request is held by this controller until the
    /// channel takes it or the block's reset line is pulsed), the pads
    /// floating.
    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Pfic::disable(S::irq());
        (void)S::disable();
        S::reset();
        S::bus_clock(false);
        SckPin::release();
        MosiPin::release();
        if constexpr (pins.miso.valid()) {
            MisoPin::release();
        }
    }

    /// Hand the NSS pad to the peripheral - an alternate output for SSOE
    /// (`hardware_output`), a pulled-up input for the multi-host
    /// arrangement (`hardware_input`) - or take it back, a floating input.
    /// Only for a program that has moved the resource to a hardware
    /// arrangement; the engine's own transactions never use it.
    template <SpiNss as = SpiNss::hardware_output>
    static void claim_nss_pad(bool on) {
        static_assert(as != SpiNss::software, "brio SpiHost: a software select has no pad to claim");
        static_assert(pins.nss.valid(), "brio SpiHost: these pins name no NSS pad");
        static_assert(as != SpiNss::hardware_output || pad_can_drive(pins.nss),
                      "brio SpiHost: SSOE DRIVES the NSS pad, and this part shares its package pin "
                      "with another pad (table 2-1, notes 4 to 7)");
        if (!on) {
            NssPin::release();
        } else if constexpr (as == SpiNss::hardware_output) {
            NssPin::function();
        } else {
            (void)NssPin::input(PinPull::up);
        }
    }

private:
    /// What the asynchronous paths read after start() has returned: the
    /// request's first eight words.
    struct Tenure {
        PinRef cs;
        PinRef dc;
        const uint8_t* cmd;
        const uint8_t* tx;
        uint8_t* rx;
        uint16_t len;
        uint8_t cmd_len;
        bool polled;
    };

    [[gnu::always_inline]] static void keep(const Request& r) {
        tenure_.cs = r.cs;
        tenure_.dc = r.dc;
        tenure_.cmd = r.cmd.get();
        tenure_.tx = r.tx.get();
        tenure_.rx = r.rx.get();
        tenure_.len = r.len;
        tenure_.cmd_len = r.cmd_len;
        tenure_.polled = r.polled;
    }

    // ---- the polled loops ---------------------------------------------------

    template <typename T>
    [[gnu::always_inline]] static uint32_t load_frame(const uint8_t* p) {
        if constexpr (sizeof(T) == 2u) {
            return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8);
        } else {
            return p[0];
        }
    }
    template <typename T>
    [[gnu::always_inline]] static void store_frame(uint8_t* p, uint32_t v) {
        p[0] = static_cast<uint8_t>(v);
        if constexpr (sizeof(T) == 2u) {
            p[1] = static_cast<uint8_t>(v >> 8);
        }
    }

    /// ONE budget of poll turns for the whole transaction: a frame's cycles
    /// per frame, two frames more for the priming and the drain - a turn is
    /// at least three instructions, so it is several times the turns the
    /// wire takes. A shift, no multiply.
    [[gnu::always_inline]] static uint32_t poll_budget(SpiClock code, SpiDataSize bits, uint32_t frames) {
        const uint32_t log2_frame = static_cast<uint32_t>(code) + 4u + (spi_frame_is_halfword(bits) ? 1u : 0u);
        return (frames + 2u) << log2_frame;
    }

    /// The loops' four specializations, each one function of the host's
    /// placement (the file header): the trampoline's instance for that
    /// specialization's lambda, the loop flattened into it. Aligned to the
    /// FETCH'S WORD: from the flash a loop's cost is a function of its
    /// address modulo four - the same instructions take 52.0 or 72.0
    /// cycles a frame at /4 as the function starts on a word or half a word
    /// into one, and no other bit of the address moves it (measured,
    /// docs/ch32x035/spi.md, "The fetch") - and -Os aligns a function to two
    /// bytes only, so without the attribute the image's layout picks the
    /// cost. Two bytes of padding at most, and the cost constants below are
    /// the aligned loops'.
    [[gnu::aligned(4)]] static bool poll_write_phase(SpiDataSize bits, const uint8_t* out, uint16_t count, uint32_t& budget) {
        if (out == nullptr) {
            return spi_frame_is_halfword(bits) ? poll_write_at<uint16_t, true>(out, count, budget)
                                               : poll_write_at<uint8_t, true>(out, count, budget);
        }
        return spi_frame_is_halfword(bits) ? poll_write_at<uint16_t, false>(out, count, budget)
                                           : poll_write_at<uint8_t, false>(out, count, budget);
    }
    [[gnu::aligned(4)]] static bool poll_duplex_phase(SpiDataSize bits, const uint8_t* out, uint8_t* in,
                                                      uint16_t count, uint32_t& budget, bool two) {
        if (out == nullptr) {
            return spi_frame_is_halfword(bits) ? poll_duplex_at<uint16_t, true>(out, in, count, budget, two)
                                               : poll_duplex_at<uint8_t, true>(out, in, count, budget, two);
        }
        return spi_frame_is_halfword(bits) ? poll_duplex_at<uint16_t, false>(out, in, count, budget, two)
                                           : poll_duplex_at<uint8_t, false>(out, in, count, budget, two);
    }
    template <typename T, bool dummy>
    [[gnu::always_inline]] static bool poll_write_at(const uint8_t* p, uint16_t count, uint32_t& budget) {
        return Hot<Placement>::run([p, count, &budget] { return poll_write<T, dummy>(p, count, budget); });
    }
    template <typename T, bool dummy>
    [[gnu::always_inline]] static bool poll_duplex_at(const uint8_t* p, uint8_t* in, uint16_t count,
                                                      uint32_t& budget, bool two) {
        return Hot<Placement>::run(
            [p, in, count, &budget, two] { return poll_duplex<T, dummy>(p, in, count, budget, two); });
    }

    /// The frame a phase sends: the next one of `p`, or with no out buffer
    /// (`dummy`) the 0xFF dummy as an immediate - no load, no stride.
    template <typename T, bool dummy>
    [[gnu::always_inline]] static uint32_t frame_at(const uint8_t* p) {
        if constexpr (dummy) {
            (void)p;
            return sizeof(T) == 2u ? 0xFFFFu : 0xFFu;
        } else {
            return load_frame<T>(p);
        }
    }

    /// The transmit-only loop (16.2.2): a frame written whenever TXE says
    /// the buffer is free, the answers left to overrun the receive buffer
    /// (16.2.7), the tail waiting TXE then not BSY and clearing RXNE and
    /// OVR with the DATAR-then-STATR read. Specialized on the out buffer's
    /// presence, so the stride is a constant and the dummy an immediate:
    /// the loop is one poll (a load, a test, a branch not taken), the
    /// frame's load, the store, the step and ONE taken branch. From the
    /// flash and its two wait states every taken branch is a fetch the
    /// core waits for: this loop takes more than the wire's 16 cycles a
    /// frame at /2, the figure its function's alignment fixes
    /// (poll_write_phase); under InRam it runs at the wire (measured,
    /// docs/ch32x035/spi.md, "The host above the wire"). `count` >= 1.
    template <typename T, bool dummy>
    [[gnu::always_inline]] static bool poll_write(const uint8_t* p, uint16_t count, uint32_t& budget) {
        SpiRegs& regs = S::regs();
        uint32_t spins = budget;
        // The loop's own count is the pointer against its end where there
        // is a buffer (the step is then the exit test's operand, not a
        // block of its own), a counter for the dummy.
        if constexpr (dummy) {
            uint32_t left = count;
            do {
                while ((regs.STATR & spi_txe) == 0u) {
                    if (--spins == 0u) {
                        budget = 0;
                        return false;
                    }
                }
                regs.DATAR = static_cast<uint16_t>(frame_at<T, dummy>(p));
            } while (--left != 0u);
        } else {
            const uint8_t* const end = p + static_cast<uint32_t>(count) * sizeof(T);
            do {
                while ((regs.STATR & spi_txe) == 0u) {
                    if (--spins == 0u) {
                        budget = 0;
                        return false;
                    }
                }
                regs.DATAR = static_cast<uint16_t>(frame_at<T, dummy>(p));
                p += sizeof(T);
            } while (p != end);
        }
        while ((regs.STATR & spi_txe) == 0u) {
            if (--spins == 0u) {
                budget = 0;
                return false;
            }
        }
        while ((regs.STATR & spi_bsy) != 0u) {
            if (--spins == 0u) {
                budget = 0;
                return false;
            }
        }
        (void)regs.DATAR;
        (void)regs.STATR;
        budget = spins;
        return true;
    }

    /// The receive loop. One frame in flight below the threshold: frame 0
    /// written; then for each frame the next one's load, the wait on RXNE,
    /// the DATAR read and the DATAR write back to back, the store. With
    /// `two`, frame 1 follows frame 0 into the buffer on TXE and every read
    /// of frame k is followed by the write of frame k + 2. `count` >= 1.
    template <typename T, bool dummy>
    [[gnu::always_inline]] static bool poll_duplex(const uint8_t* p, uint8_t* in, uint16_t count, uint32_t& budget,
                                                   bool two) {
        constexpr uint32_t step = dummy ? 0u : sizeof(T);
        SpiRegs& regs = S::regs();
        uint32_t spins = budget;
        uint32_t to_write = count;
        uint32_t to_read = count;
        regs.DATAR = static_cast<uint16_t>(frame_at<T, dummy>(p));   // the shifter is idle: frame 0 moves at once
        p += step;
        --to_write;
        if (two && to_write != 0u) {
            while ((regs.STATR & spi_txe) == 0u) {
                if (--spins == 0u) {
                    budget = 0;
                    return false;
                }
            }
            regs.DATAR = static_cast<uint16_t>(frame_at<T, dummy>(p));
            p += step;
            --to_write;
        }
        // The frames with one still to write after them, then the tail that
        // only reads: no test of the write count inside the steady loop.
        while (to_write != 0u) {
            const uint32_t next = frame_at<T, dummy>(p);
            // The load belongs INSIDE the wire time: the compiler would sink
            // it past the poll, onto the bus's dead time.
            __asm__ volatile("" : : "r"(next));
            while ((regs.STATR & spi_rxne) == 0u) {
                if (--spins == 0u) {
                    budget = 0;
                    return false;
                }
            }
            const uint32_t got = regs.DATAR;
            regs.DATAR = static_cast<uint16_t>(next);
            p += step;
            --to_write;
            store_frame<T>(in, got);
            in += sizeof(T);
            --to_read;
        }
        while (to_read != 0u) {
            while ((regs.STATR & spi_rxne) == 0u) {
                if (--spins == 0u) {
                    budget = 0;
                    return false;
                }
            }
            store_frame<T>(in, regs.DATAR);
            in += sizeof(T);
            --to_read;
        }
        budget = spins;
        return true;
    }

    /// A polled flag that never came: the status, and the buffer flushed so
    /// the next transaction does not read this one's frame.
    static void stalled() {
        status_ = spi_stalled;
        S::flush_rx();
    }

    // ---- the engines --------------------------------------------------------

    /// Do the engines take a 16-bit beat?
    static constexpr bool engines_carry_halfwords = [] {
        if constexpr (TxEngine::present) {
            return sizeof(typename TxEngine::element) >= 2u && sizeof(typename RxEngine::element) >= 2u;
        } else {
            return false;
        }
    }();

    /// Can the engines carry a block of these frames between these buffers?
    /// Every block in 8-bit frames; one in 16-bit frames when they take
    /// half-words and both buffers sit on a half-word boundary (an absent
    /// buffer is the cell, aligned).
    [[gnu::always_inline]] static bool beats_serve(SpiDataSize bits, const uint8_t* a, const uint8_t* b) {
        if (!spi_frame_is_halfword(bits)) {
            return true;
        }
        if constexpr (engines_carry_halfwords) {
            const uint32_t odd = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(a)) |
                                 static_cast<uint32_t>(reinterpret_cast<uintptr_t>(b));
            return (odd & 1u) == 0u;
        } else {
            (void)a;
            (void)b;
            return false;
        }
    }
    /// The data phase's block.
    [[gnu::always_inline]] static bool dma_serves(const Request& r) {
        return beats_serve(r.bits, r.tx.get(), r.rx.get());
    }

    /// Does an asynchronous request repay the engines (the arithmetic above
    /// SpiHostCosts, folded from the two frame costs kept for the code in
    /// force), and can they carry it? Its command phase counts as frames
    /// written, its data phase as frames read or written.
    [[gnu::always_inline]] static bool repays_engines(const Request& r) {
        const bool read = r.rx.get() != nullptr;
        const uint32_t cost = static_cast<uint32_t>(r.cmd_len) * write_frame_cost_ +
                              static_cast<uint32_t>(r.len) * (read ? read_frame_cost_ : write_frame_cost_);
        if (cost < (read ? async_read_budget_cycles : async_write_budget_cycles)) {
            return false;
        }
        return r.len != 0u ? dma_serves(r) : beats_serve(r.bits, r.cmd.get(), nullptr);
    }

    /// An asynchronous request onto the engines - once a transaction, kept
    /// out of the vectors' way. Its command phase rides them as a block of
    /// its own when it is the whole request or would repay a block alone,
    /// and runs polled here otherwise; then the data block. False: dma_isr()
    /// ends it. True only when the polled command phase stalled.
    static bool start_engines(const Request& r, SpiClock code) {
        keep(r);
        step_ = spi_frame_is_halfword(r.bits) ? 2u : 1u;
        if (r.cmd_len != 0u) {
            const bool cmd_block =
                r.len == 0u || (static_cast<uint32_t>(r.cmd_len) * write_frame_cost_ >= async_write_budget_cycles &&
                                beats_serve(r.bits, r.cmd.get(), nullptr));
            if (cmd_block) {
                in_cmd_ = true;
                launch_dma(r.cmd.get(), nullptr, r.cmd_len);   // the echo discarded
                return false;
            }
            uint32_t budget = poll_budget(code, r.bits, r.cmd_len);
            if (!poll_write_phase(r.bits, r.cmd.get(), r.cmd_len, budget)) {
                r.cs.set();
                stalled();
                return true;
            }
            r.dc.set();
        }
        in_cmd_ = false;
        launch_dma(r.tx.get(), r.rx.get(), r.len);
        return false;
    }

    /// A block onto the engines - once a block, not inlined: a vector
    /// chains the data block from here.
    [[gnu::noinline]] static void launch_dma(const uint8_t* tx, uint8_t* rx, uint16_t count) {
        dma_done_ = false;
        dma_active_ = true;
        if constexpr (engines_carry_halfwords) {
            if (step_ == 2u) {
                launch_beats<uint16_t>(tx, rx, count);
                return;
            }
        }
        launch_beats<uint8_t>(tx, rx, count);
    }

    /// The RECEIVE channel first: its request rises only when a frame has
    /// come back, and the transmit side is what starts the clock.
    template <typename T>
    static void launch_beats(const uint8_t* out, uint8_t* in, uint16_t count) {
        if constexpr (has_dma) {
            T* const rx = reinterpret_cast<T*>(in);
            const T* const tx = reinterpret_cast<const T*>(out);
            if (rx != nullptr) {
                (void)RxEngine::start(std::span<T>(rx, count));
            } else {
                (void)RxEngine::start_discard(reinterpret_cast<T*>(&rx_sink_), count);
            }
            S::dma_requests(false, true);
            if (tx != nullptr) {
                (void)TxEngine::start(std::span<const T>(tx, count));
            } else {
                (void)TxEngine::start_fixed(reinterpret_cast<const T*>(&tx_dummy_), count);
            }
            S::dma_requests(true, true);
        }
    }

    /// A command block ended: D/C up, then the data block onto the engines,
    /// or - no data phase - the transaction's end. Interrupt context.
    static bool next_block() {
        in_cmd_ = false;
        tenure_.dc.set();
        if (tenure_.len == 0u) {
            return finish_dma(spi_ok);
        }
        if constexpr (has_dma) {
            S::dma_requests(false, false);
            (void)TxEngine::complete();   // the receive block's end proves the transmit one's
        }
        launch_dma(tenure_.tx, tenure_.rx, tenure_.len);
        return false;
    }

    /// One exit for a tenure on the engines. True when the vector's caller
    /// should post completion: an asynchronous tenure, its select released.
    static bool finish_dma(uint8_t st) {
        if constexpr (has_dma) {
            S::dma_requests(false, false);
            if (st != spi_ok) {
                status_ = st;
                (void)TxEngine::abandon();
                RxEngine::stop();
                S::flush_rx();   // a frame the stopped channel never took
            } else {
                // The receive block completing is proof the transmit one
                // did: every frame that came back was clocked out first.
                (void)TxEngine::complete();
            }
        }
        in_cmd_ = false;
        dma_active_ = false;
        dma_done_ = true;
        if (!tenure_.polled) {
            tenure_.cs.set();
            return true;
        }
        return false;
    }

    /// The polled request's wait on the DMA completion - bounded (the
    /// slowest frame is 256 x 16 HCLK cycles; the budget scales).
    static void spin_dma() {
        if constexpr (has_dma) {
            uint32_t spins = 200'000u + 6'000u * static_cast<uint32_t>(tenure_.len);
            while (!dma_done_ && spins-- != 0u) {
            }
            if (!dma_done_) {
                S::dma_requests(false, false);
                (void)TxEngine::abandon();
                RxEngine::stop();
                dma_active_ = false;
                status_ = spi_dma_fault;
                S::flush_rx();
            }
        }
    }

    // ---- the configuration --------------------------------------------------

    [[gnu::always_inline]] static SpiClock clamp(SpiClock c) {
        return static_cast<uint8_t>(c) < floor_code_ ? static_cast<SpiClock>(floor_code_) : c;
    }

    static SpiConfig boot_config() {
        SpiConfig c{};
        c.role = SpiRole::host;
        c.mode = SpiMode::mode0;
        c.bits = SpiDataSize::bits8;
        c.clock = ceiling_ ? *ceiling_ : SpiClock::div16;
        c.nss = SpiNss::software;
        c.open_drain = open_drain_;
        c.high_speed_read = high_speed_ && c.clock == SpiClock::div2;
        // The DMA requests are NOT part of the applied configuration:
        // launch_dma() raises them around the engines.
        return c;
    }

    /// SPE up over the applied configuration, and the control word the
    /// per-request compare is made against kept beside it.
    static void enable_applied() {
        ctlr1_ = static_cast<uint16_t>(spi_ctlr1_of(applied_) | spi_spe);
        S::regs().CTLR1 = ctlr1_;
        Request r{};
        r.clock = applied_.clock;
        r.mode = applied_.mode;
        r.bits = applied_.bits;
        request_key_ = request_key(r);
        costs_for(applied_.clock, applied_.bits);
    }

    /// The two frame costs the short-request arithmetic folds a request
    /// from, kept for the code and width in force: computed where CTLR1 is
    /// written, never per request.
    static void costs_for(SpiClock c, SpiDataSize bits) {
        if constexpr (has_dma) {
            read_frame_cost_ = static_cast<uint16_t>(spi_polled_frame_cost(costs, c, bits, true));
            write_frame_cost_ = static_cast<uint16_t>(spi_polled_frame_cost(costs, c, bits, false));
        } else {
            (void)c;
            (void)bits;
        }
    }

    /// The CTLR1 bits a request decides: CPHA and CPOL (the mode), BR (the
    /// rate), DFF (the width).
    static constexpr uint16_t request_bits_mask = spi_cpha | spi_cpol | spi_br_mask | spi_dff;
    static constexpr uint16_t request_bits(SpiMode m, SpiClock c, SpiDataSize bits) {
        uint16_t v = static_cast<uint16_t>(static_cast<uint16_t>(c) << 3);
        if (spi_mode_cpha(m)) { v |= spi_cpha; }
        if (spi_mode_cpol(m)) { v |= spi_cpol; }
        if (bits == SpiDataSize::bits16) { v |= spi_dff; }
        return v;
    }

    /// The request's clock, mode and width as one word: the three bytes lie
    /// together behind cs_setup_us at a word boundary (the static_assert
    /// above), so one load and a shift read them.
    [[gnu::always_inline]] static uint32_t request_key(const Request& r) {
        uint32_t w;
        __builtin_memcpy(&w, __builtin_assume_aligned(&r.cs_setup_us, 4), sizeof w);
        return w >> 8;
    }

    /// Put the peripheral where this request wants it. The compare is ONE
    /// WORD against the last request's; equal, nothing moves. Different,
    /// the fields are folded into the control word and compared with the
    /// one in force, and a changed word is written with the drain and the
    /// disable/enable pair 16.3.1 asks for DFF - and HSCR follows the code:
    /// the high-speed read on only at /2 (16.3.8), and dropped BEFORE any
    /// other code is written, since 16.3.1 makes the bit change what every
    /// other code divides by.
    [[gnu::always_inline]] static void apply(const Request& r, SpiClock code) {
        const uint32_t key = request_key(r);
        if (key != request_key_) {
            reconfigure(key, r.mode, code, r.bits);
        }
    }
    static void reconfigure(uint32_t key, SpiMode m, SpiClock c, SpiDataSize bits) {
        request_key_ = key;
        const uint16_t want = static_cast<uint16_t>((ctlr1_ & ~request_bits_mask) | request_bits(m, c, bits));
        if (want == ctlr1_) {
            return;
        }
        applied_.mode = m;
        applied_.clock = c;
        applied_.bits = bits;
        const bool hs = high_speed_ && c == SpiClock::div2;
        (void)S::disable();                                            // the drain, then SPE down
        if (hs != S::high_speed_read()) {
            S::high_speed_read_raw(false);                             // never on under another code
        }
        S::regs().CTLR1 = static_cast<uint16_t>(want & ~spi_spe);      // the new word, SPE still clear
        if (hs) {
            S::high_speed_read_raw(true);
        }
        applied_.high_speed_read = hs;
        S::regs().CTLR1 = want;                                        // and up
        ctlr1_ = want;
        costs_for(c, bits);
    }

    // ---- the state ----------------------------------------------------------

    static inline Tenure tenure_{};
    static inline uint8_t step_ = 1;    ///< bytes per frame in the buffers
    /// A command block is on the engines; its end puts the data block there.
    static inline bool in_cmd_ = false;
    static inline uint8_t status_ = spi_ok;
    /// A polled frame's cost at the code in force, read and written
    /// (costs_for()): what start() folds an asynchronous request from.
    static inline uint16_t read_frame_cost_ = 0;
    static inline uint16_t write_frame_cost_ = 0;
    static inline volatile bool dma_done_ = false;
    static inline volatile bool dma_active_ = false;
    /// The dummy frame of the polled loops (read at a stride of zero), and
    /// the engines' discard and dummy cells - in SRAM, half-words so either
    /// beat moves them: a cell in flash costs the transmit channel the
    /// flash's wait states on every frame.
    static constexpr uint8_t dummy_bytes_[2] = {0xFFu, 0xFFu};
    static inline uint16_t rx_sink_ = 0;
    static inline uint16_t tx_dummy_ = 0xFFFFu;
    static inline SpiConfig applied_{};
    static inline uint16_t ctlr1_ = 0;          ///< CTLR1 as it stands, SPE included
    static inline uint32_t request_key_ = 0;    ///< the last request's clock, mode and width as one word
    static inline uint8_t floor_code_ = 0;
    static inline uint32_t hclk_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<SpiClock> ceiling_{};
    static inline bool open_drain_ = false;
    static inline bool high_speed_ = false;
    static inline DelayRate cs_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/**
 * What this die does as a CLIENT that ANSWERS, mode by mode and order by
 * order, as test_x035_spi letters r and w measured it under the CH32V006's
 * host: exact in every mode MSb first; LSb first its MISO never moves
 * (16.3.1: "LSB is only supported by SPI as host"). 16.2.3's "mode 2 or
 * mode 3" does not hold on this die. A client that only LISTENS is served
 * in every mode and both orders (measured too).
 */
constexpr bool spi_client_answers(SpiMode m, bool lsb_first) {
    (void)m;
    return !lsb_first;
}

/**
 * SpiClient<n, pins>
 *
 * The other end of the wire: SCK, MOSI and NSS are inputs, the host sets
 * the pace, and the only thing this side controls is WHAT it has ready to
 * shift out when the next clock arrives.
 *
 * One frame ahead (`frames_ahead`). With no FIFO the transmit buffer holds
 * exactly the next answer: the buffer moves into the shifter "when the
 * first slave receive sample edge appears in SCK" and TXE rises then
 * (16.2.3) - the moment to write the next one, which then has the rest of
 * the frame to land. A client that falls behind does not stall the bus:
 * what the host reads is whatever the shifter held, a wrong answer and
 * never a missing one.
 *
 * The NSS pad is the window: the chapter publishes no flag for a client's
 * transaction end, so the runs read the pad (`selected()`). `drive_output`
 * makes this client a dark listener: MISO is handed to the peripheral only
 * for the window it answers in, a floating input otherwise.
 */
template <uint8_t n, SpiPins pins = spi1_default_pins>
class SpiClient {
    using S = Spi<n>;

    static_assert(spi_pins_valid(pins),
                  "brio SpiClient: these SPI pads are not a link - SCK is required, every pad must "
                  "be its column's own (afio.hpp, SPI1_RM) and bonded by this part");
    static_assert(pins.mosi.valid(), "brio SpiClient: a client listens on MOSI");
    static_assert(!pins.miso.valid() || pad_can_drive(pins.miso),
                  "brio SpiClient: the client DRIVES MISO, and this part shares that pad's package "
                  "pin with another pad - the datasheet forbids it as an output (table 2-1, notes "
                  "4 to 7; parts/<part>.hpp's port_twinned)");

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

    /// Is this configuration one the client serves? A client that answers
    /// (MISO driven, full duplex) only where spi_client_answers() says this
    /// die does; a listener in every mode and order.
    static constexpr bool config_served(const Config& c) {
        const bool answers = c.drive_output && pins.miso.valid() &&
                             (c.direction == SpiDirection::full_duplex ||
                              c.direction == SpiDirection::half_duplex_out);
        return !answers || spi_client_answers(c.mode, c.lsb_first);
    }

    /// How many answers must be queued ahead of the frame the host is about
    /// to clock: ONE here - no FIFO, one buffer.
    static constexpr uint8_t frames_ahead = 1;

    /// Bring the instance up as a client. The bus clock is required though
    /// the shifter runs on the host's SCK. False when the configuration is
    /// refused - by the chapter, or by what this die measured as a client
    /// (config_served).
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        if (!config_served(cfg)) {
            return false;
        }
        Pfic::disable(S::irq());
        S::bus_clock(true);
        S::reset();
        Afio::remap(Remap::spi1, pins.remap);
        if (!S::configure(config_of(cfg))) {
            return false;
        }
        // Inputs first, the output last.
        SckPin::input();
        MosiPin::input();
        if constexpr (has_nss_pad) {
            if (cfg.nss == SpiNss::hardware_input) {
                (void)NssPin::input(PinPull::up);   // table 8-4: floating, pulled up or down
            }
        }
        drive_output(cfg.drive_output);
        S::flush_rx();
        Pfic::enable(S::irq());
        return true;
    }

    /// Hand the answer line to the peripheral, or take it back (a floating
    /// input, driving nothing).
    static void drive_output(bool on) {
        if constexpr (pins.miso.valid()) {
            if (on) { MisoPin::function(); } else { MisoPin::release(); }
        } else {
            (void)on;
        }
    }

    /// SPE up and the first answer in the buffer before the host's clock can
    /// arrive. Call it with NSS still high.
    static void enable(uint16_t first) {
        S::enable();
        write(first);
    }
    /// SPE up with the transmit buffer EMPTY: the opening of a run, which
    /// writes its own first answer.
    static void enable() { S::enable(); }
    static bool disable() { return S::disable(); }

    /// Load the next frame to shift out; on TXE, for the one-ahead rule.
    static void write(uint16_t v) { S::data(v); }
    static bool writable() { return S::txe(); }

    /// One received frame, or nothing. Reading DATAR clears RXNE.
    static std::optional<uint16_t> poll() {
        if (!S::rxne()) {
            return {};
        }
        return static_cast<uint16_t>(S::data() & spi_frame_mask(bits_));
    }

    /// Is the host holding NSS low right now? A live pad read; always false
    /// without an NSS pad.
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

    // ---- the polled runs ---------------------------------------------------
    //
    // A client's unit is the run: the answers to a transaction, or what it
    // brings, served polled in one call - the per-frame verbs above being
    // its degenerate case. What bounds a run is the one transmit buffer in
    // front of the shifter: TXE rises as frame k enters the shifter, and
    // frame k + 1's answer may be written from then to frame k + 1's first
    // edge, most of a frame of window. The receive buffer is ONE frame
    // deep, so a run that reads must read each frame inside the next one.
    // The select pad is the window: a run waits for the pad to fall before
    // it takes a high pad for the close, reading it once for every four
    // polls of the flag; a run without an NSS pad ends on its count or its
    // budget alone. They are always inline: the caller places them.

    /// How a run ended.
    enum class RunEnd : uint8_t {
        complete,   ///< every frame of the run moved
        closed,     ///< the host raised the select (the NSS pad read high)
        stalled,    ///< the spin budget ran out first
    };

    /// What a run did: `count` frames - answers written for answer(),
    /// frames received for exchange() and receive() - and how it ended.
    struct Run {
        uint16_t count;
        RunEnd end;
    };

    /**
     * Answer a transaction with `out`, polled on TXE alone, reading
     * nothing. Call it with SPE up (`enable()`), the buffer empty and the
     * select still high: the first answer goes into the buffer at once,
     * every later one on the TXE of the frame before it. Ends when the host
     * raises the select; the answers past the run's end carry what the
     * shifter is left with. The receive buffer overruns from the second
     * frame (nothing is read) and the tail clears it.
     */
    template <typename T>
    [[gnu::always_inline]] static Run answer(std::span<const T> out, uint32_t spins) {
        static_assert(run_frame<T>, "a frame of this block is eight bits or sixteen (DFF): uint8_t or uint16_t");
        SpiRegs& regs = S::regs();
        const T* const first = out.data();
        const T* const end = first + out.size();
        const T* p = first;
        RunEnd how = RunEnd::closed;
        if (p != end) {
            regs.DATAR = *p++;
        }
        if (!await_select(spins)) {
            how = RunEnd::stalled;
            goto tail;
        }
        for (; p != end; ++p) {
            const T next = *p;
            if (!await<spi_txe>(spins, how)) {
                goto tail;
            }
            regs.DATAR = next;
        }
        if constexpr (has_nss_pad) {
            while (pad_low()) {
                if (--spins == 0u) {
                    how = RunEnd::stalled;
                    break;
                }
            }
        } else {
            how = RunEnd::complete;
        }
    tail:
        S::flush_rx();
        return {static_cast<uint16_t>(p - first), how};
    }

    /**
     * EXCHANGE: answer with `out` and keep what the host sends in `in`, the
     * run's length the shorter of the two. The answers go as answer()'s;
     * each turn then reads the frame the TXE belonged to, on its RXNE. Ends
     * `complete` when the last frame is in, `closed` when the host raised
     * the select first (what arrived before is in `in`), `stalled` on the
     * budget. The receive side is cleared at the start, the select high.
     */
    template <typename T>
    [[gnu::always_inline]] static Run exchange(std::span<const T> out, std::span<T> in, uint32_t spins) {
        static_assert(run_frame<T>, "a frame of this block is eight bits or sixteen (DFF): uint8_t or uint16_t");
        const uint32_t len = out.size() < in.size() ? out.size() : in.size();
        if (len == 0u) {
            return {0, RunEnd::complete};
        }
        SpiRegs& regs = S::regs();
        S::flush_rx();
        const T* p = out.data();
        const T* const p_end = p + len;
        T* const first = in.data();
        T* const q_end = first + len;
        T* q = first;
        RunEnd how = RunEnd::complete;
        regs.DATAR = *p++;
        if (!await_select(spins)) {
            return {0, RunEnd::stalled};
        }
        for (; p != p_end; ++p, ++q) {
            const T next = *p;
            if (!await<spi_txe>(spins, how)) {
                goto drain;
            }
            regs.DATAR = next;
            if (!await<spi_rxne>(spins, how)) {
                goto drain;
            }
            *q = static_cast<T>(regs.DATAR);
        }
        if (!await<spi_rxne>(spins, how)) {
            goto drain;
        }
        *q++ = static_cast<T>(regs.DATAR);
        return {static_cast<uint16_t>(q - first), RunEnd::complete};
    drain:
        if (q != q_end && (regs.STATR & spi_rxne) != 0u) {
            *q++ = static_cast<T>(regs.DATAR);
        }
        return {static_cast<uint16_t>(q - first), how};
    }

    /**
     * RECEIVE a run into `in`, polled on RXNE alone, writing nothing. The
     * receive buffer is NOT flushed at the start, so a frame standing in it
     * when the run began is the run's first. Ends as exchange() does.
     */
    template <typename T>
    [[gnu::always_inline]] static Run receive(std::span<T> in, uint32_t spins) {
        static_assert(run_frame<T>, "a frame of this block is eight bits or sixteen (DFF): uint8_t or uint16_t");
        SpiRegs& regs = S::regs();
        T* const first = in.data();
        T* const q_end = first + in.size();
        T* q = first;
        RunEnd how = RunEnd::complete;
        if ((regs.STATR & spi_rxne) == 0u && !await_select(spins)) {
            return {0, RunEnd::stalled};
        }
        for (; q != q_end; ++q) {
            if (!await<spi_rxne>(spins, how)) {
                if ((regs.STATR & spi_rxne) != 0u) {
                    *q++ = static_cast<T>(regs.DATAR);
                }
                break;
            }
            *q = static_cast<T>(regs.DATAR);
        }
        return {static_cast<uint16_t>(q - first), how};
    }

    /// The ISR body: the raised-and-enabled sources, for the app's glue.
    [[gnu::always_inline]] static uint32_t isr() { return S::isr(); }

    static void rxne_interrupt(bool on) { S::rxne_interrupt(on); }
    static void txe_interrupt(bool on) { S::txe_interrupt(on); }
    static void error_interrupt(bool on) { S::error_interrupt(on); }

    static SpiDataSize bits() { return bits_; }

    /// The block handed back: the drain, the reset pulse before the gate
    /// closes (the release contract, as SpiHost::release()), the pads
    /// floating.
    static void release() {
        Pfic::disable(S::irq());
        (void)S::disable();
        S::reset();
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

    template <typename T>
    static constexpr bool run_frame = std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>;

    /// The NSS pad read low, the window open (yes at once without a pad).
    [[gnu::always_inline]] static bool pad_low() {
        if constexpr (has_nss_pad) {
            return (NssPin::P::regs().INDR & NssPin::mask) == 0u;
        } else {
            return true;
        }
    }

    [[gnu::always_inline]] static bool await_select(uint32_t& spins) {
        while (!pad_low()) {
            if (--spins == 0u) {
                return false;
            }
        }
        return true;
    }

    /// Poll STATR until `flag` stands: true. False when the host raised the
    /// select or the budget ran out, `how` saying which. Four polls a look:
    /// the pad and the budget are asked once every four reads, so the read
    /// that sees the flag is a load, a test and a branch from the one
    /// before it.
    template <uint16_t flag>
    [[gnu::always_inline]] static bool await(uint32_t& spins, RunEnd& how) {
        SpiRegs& regs = S::regs();
        for (;;) {
            if ((regs.STATR & flag) != 0u) { return true; }
            if ((regs.STATR & flag) != 0u) { return true; }
            if ((regs.STATR & flag) != 0u) { return true; }
            if ((regs.STATR & flag) != 0u) { return true; }
            if constexpr (has_nss_pad) {
                if (!pad_low()) {
                    how = RunEnd::closed;
                    return false;
                }
            }
            if (--spins == 0u) {
                how = RunEnd::stalled;
                return false;
            }
        }
    }

    static inline SpiDataSize bits_ = SpiDataSize::bits8;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// 16.3.1's BR table at the 48 MHz the HSI gives; table 3-19's 24 MHz.
static_assert(spi_sck_hz(48'000'000UL, SpiClock::div2) == 24'000'000UL);
static_assert(spi_sck_hz(48'000'000UL, SpiClock::div256) == 187'500UL);
static_assert(spi_rate_for(48'000'000UL, 24'000'000UL) == SpiClock::div2);
static_assert(spi_rate_for(48'000'000UL, 23'999'999UL) == SpiClock::div4);
static_assert(spi_rate_for(48'000'000UL, 1'000'000UL) == SpiClock::div64);
static_assert(!spi_rate_for(48'000'000UL, 187'499UL).has_value());
static_assert(!spi_rate_for(0, 1'000'000UL).has_value());

static_assert(spi_frame_cycles(SpiClock::div2, SpiDataSize::bits8) == 16u);
static_assert(spi_frame_cycles(SpiClock::div256, SpiDataSize::bits16) == 4096u);
// The write-ahead threshold under the default hold-off (380 + 57 = 437
// cycles): two frames in flight from /64 on 8-bit frames and from /32 on
// 16-bit ones - the same codes for any hold-off from 199 to 454 cycles; a
// hold-off of 198 moves both one code faster, one of 600 one code slower.
static_assert(spi_write_ahead_from(SpiDataSize::bits8, spi_default_hold_off_cycles) == SpiClock::div64);
static_assert(spi_write_ahead_from(SpiDataSize::bits16, spi_default_hold_off_cycles) == SpiClock::div32);
static_assert(spi_write_ahead_from(SpiDataSize::bits8, 199u) == SpiClock::div64);
static_assert(spi_write_ahead_from(SpiDataSize::bits8, 454u) == SpiClock::div64);
static_assert(spi_write_ahead_from(SpiDataSize::bits8, 198u) == SpiClock::div32);
static_assert(spi_write_ahead_from(SpiDataSize::bits16, 198u) == SpiClock::div16);
static_assert(spi_write_ahead_from(SpiDataSize::bits8, 600u) == SpiClock::div128);
static_assert(spi_write_ahead_from(SpiDataSize::bits16, 600u) == SpiClock::div64);

// The short asynchronous request (the arithmetic above SpiHostCosts): from
// the flash, E - F = 660 cycles for a read and 705 for a write; a byte
// frame read costs 52 at /2 and /4 (the loop's turn; the wire and the dead
// bus at /4), 84 at /8, 148 at /16, 276 at /32, 532 at /64; written, 24 at
// /2 (the loop's turn) and the wire from /4. So a read rides the engines
// from 13, 13, 8, 5, 3 and 2 frames at /2 to /64 and from one at /128, a
// write from 30, 23, 12, 6, 3 and 2; a polled read from 660 / 20, 33
// frames (dma_min_frames_polled).
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div2, SpiDataSize::bits8, true) == 13u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div4, SpiDataSize::bits8, true) == 13u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div8, SpiDataSize::bits8, true) == 8u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div16, SpiDataSize::bits8, true) == 5u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div32, SpiDataSize::bits8, true) == 3u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div64, SpiDataSize::bits8, true) == 2u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div128, SpiDataSize::bits8, true) == 1u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div2, SpiDataSize::bits8, false) == 30u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div4, SpiDataSize::bits8, false) == 23u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div8, SpiDataSize::bits8, false) == 12u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div16, SpiDataSize::bits8, false) == 6u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div32, SpiDataSize::bits8, false) == 3u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div64, SpiDataSize::bits8, false) == 2u);
static_assert(spi_async_min_frames(spi_costs_in_flash, SpiClock::div256, SpiDataSize::bits8, true) == 1u);

// The Motorola numbering (the file header).
static_assert(!spi_mode_cpol(SpiMode::mode1) && spi_mode_cpha(SpiMode::mode1));
static_assert(spi_mode_cpol(SpiMode::mode2) && !spi_mode_cpha(SpiMode::mode2));

// The refusals spi_config_valid() owes the chapter.
static_assert(spi_config_valid(SpiConfig{}));
static_assert(!spi_config_valid(SpiConfig{.direction = SpiDirection::half_duplex_out, .crc = true}));
static_assert(!spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0}));
static_assert(!spi_config_valid(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));
static_assert(!spi_config_valid(SpiConfig{.clock = SpiClock::div4, .high_speed_read = true}));
static_assert(spi_config_valid(SpiConfig{.clock = SpiClock::div2, .high_speed_read = true}));
static_assert(!spi_config_valid(SpiConfig{.role = SpiRole::client, .clock = SpiClock::div2, .high_speed_read = true}));

// The columns (8.3.2.1) and the link rule.
static_assert(spi1_default_pins.nss == Pad{'A', 4} && spi1_default_pins.sck == Pad{'A', 5} &&
              spi1_default_pins.miso == Pad{'A', 6} && spi1_default_pins.mosi == Pad{'A', 7});
static_assert(spi1_pins_for(3).sck == Pad{'C', 5} && spi1_pins_for(3).remap == 3u);
static_assert(!spi_pins_valid(SpiPins{.sck = {'A', 5}, .mosi = {'A', 6}}));   // MOSI not the column's
static_assert(!spi_pins_valid(SpiPins{.mosi = {'A', 7}}));                   // no SCK
static_assert(!spi_pins_valid(SpiPins{.sck = {'A', 5}, .remap = 4}));        // no such code

// The control words: mode 3 host at /16 under software select; ODEN.
static_assert(spi_ctlr1_of(SpiConfig{.mode = SpiMode::mode3, .clock = SpiClock::div16}) ==
              (spi_cpha | spi_cpol | spi_mstr | (3u << 3) | spi_ssm | spi_ssi));
static_assert(spi_ctlr2_of(SpiConfig{.nss = SpiNss::hardware_output, .dma_receive = true, .open_drain = true}) ==
              (spi_ssoe | spi_rxdmaen | spi_oden));

} // namespace brio
