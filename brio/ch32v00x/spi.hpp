/*
 * spi.hpp
 *
 * The SPI of the CH32V00x (RM ch. 16), for a program that talks to a
 * device over SPI - a display, a converter, a memory - or is the device
 * at the other end. One instance, both roles, 8- or 16-bit frames, the
 * four modes, a software or hardware chip select, hardware CRC, two DMA
 * requests: the STM32F1's SPI with one register of its own (HSCR, a
 * "high-speed read" mode) and without the I2S half. Three layers, the
 * arrangement every brio SPI driver has:
 *
 *  - `Spi<n>` is the resource: the register block, its gate and reset,
 *    the configuration under the rules the chapter states, the data
 *    and status verbs, the ISR body that reads the raised-and-enabled
 *    sources. It decides nothing.
 *  - `SpiHost<n, pins, TxEngine, RxEngine, hold_off_cycles, Placement>`
 *    is the engine util/spi_bus.hpp's SpiBus drives (the engine is the
 *    driver layer that runs one transaction on the wire; SpiBus is the
 *    arbiter, an active object - a component of a brio program with its
 *    own event queue, served one event at a time by the kernel - that
 *    queues the clients' requests and gives the host one at a time).
 *    The Request's field names are the same on every stratum (chip
 *    select, D/C, a command phase, a data phase with optional out and
 *    in, the per-request mode/rate/frame size, the polled or ISR-pumped
 *    completion), so an application that fills them by name over SpiBus
 *    on another family's board runs here unchanged; their order is this
 *    host's own.
 *  - `SpiClient<n, pins>` is the other end of the wire: the per-frame
 *    verbs, an ISR body, and the polled runs a transaction is served by
 *    (answer, exchange, receive; a run is the frames of one transaction
 *    served in one call); which protocol it serves is the application's.
 *
 * The smallest use, a host on the default pads under its arbiter, as
 * docs/ch32v00x/spi.md spells it:
 *
 *     using Bus = brio::SpiHost<1>;                         // the default pads
 *     using Spi = brio::SpiBus<Bus, P, 4>;                  // the arbiter AO
 *     using Cs = brio::Pin<'C', 3>;
 *
 *     Bus::init(clock, 8'000'000);                          // a ceiling for the whole bus
 *     Cs::output(true);
 *
 *     Bus::Request r{};
 *     r.cs = Cs::ref();
 *     r.cmd = brio::lend<brio::Lease::reply>(cmd);  r.cmd_len = 1;
 *     r.tx = brio::lend<brio::Lease::reply>(pixels); r.len = 64;
 *     r.clock = brio::SpiClock::div4;                       // at 48 MHz: clamped to div8, 6 MHz
 *     r.mode = brio::SpiMode::mode0;
 *     r.reply = brio::reply_to<Display, brio::SpiDone>();
 *     brio::post<Spi>(r);
 *
 *     BRIO_CH32_VECTOR(spi1_handler) {
 *         if (Bus::isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
 *     }
 *
 * The ceiling clamps a request's clock and never speeds it up: HCLK/4
 * is 12 MHz at 48 MHz, over the 8 MHz ceiling, so this request runs at
 * HCLK/8, 6 MHz.
 *
 * Who calls what. `init()` runs in main, after the clock task's init()
 * (the ceiling's BR code and the CS setup timing are computed from the
 * rate the Clock tag claims) and before the kernel's run(). The chip
 * select is a GPIO the program configures as an output, high, before
 * the first request, and names in each request; the host drives it low
 * around the transaction and never claims the NSS pad. `start()` is the
 * arbiter's, from its dispatch in main-loop context: a polled request
 * completes inside it, each frame's flag waited for under one bounded
 * budget for the whole transaction (spi_stalled when it runs out), so it
 * holds the main loop for the transaction's wire time; an asynchronous
 * one returns once the select is asserted and its cs_setup_us has been
 * waited out (a busy wait, as on the polled path), and the rest is
 * pumped from the vector. `isr()`,
 * and with engines `dma_isr()`, are the bodies of the vectors the
 * program binds (spi1_handler; dma1_channel2_handler and
 * dma1_channel3_handler) and are called from nowhere else; true from
 * either is the edge the program posts TransferDone on. No path of the
 * host masks interrupts; it disables its own PFIC line only inside
 * init(), recover() and release(). The request's three buffers are lent
 * (`Borrowed<..., Lease::reply>`): the client keeps them valid and
 * untouched from the post until its SpiDone reply arrives; the
 * descriptor itself is lent for the call, read through the reference,
 * the asynchronous tenure (the span from start() to the completion)
 * copied into the host. On the CH32V003 an engined transaction keeps the
 * core awake to its end (that part's idle() does not sleep over a
 * working DMA channel, ch32v00x/dma.hpp's any_working()), and the pump
 * with two frames in flight overruns when that part's loop sleeps
 * between frames (the hold-off below, and the gap list at the end).
 *
 * What this silicon has, and has not. There is no FIFO: one transmit
 * buffer, one receive buffer, one shift register (figure 16-1), so TXE
 * means "the buffer moved into the shifter" and RXNE "a frame has
 * been shifted both ways" (16.2.2). A frame written while another
 * shifts waits in the buffer and follows it with no gap on SCK - "if
 * the TXE flag is set, fill the data register, maintain the complete
 * data flow" - which is what the host's transmit-only loop and its
 * write-ahead pump build on; and the receive buffer is one frame deep,
 * which is what bounds them (16.2.7, OVR: the arithmetic above the
 * host engine). The host's pump runs on RXNE, because a frame that
 * came back is the one witness that the bus is idle for the next. The
 * frame is 8 or 16 bits (DFF), and both DFF and CRCEN "can only be
 * written when SPE is 0" (16.3.1), which is what makes apply() pay a
 * disable/enable pair when a request changes the size. The baud rate
 * is HCLK over a power of two, /2 to /256, and the manual promises SCK
 * up to HCLK/2 - 24 MHz at the 48 MHz this stratum runs at. The chip
 * select the engine uses is a GPIO the Request carries (SSM set, SSI
 * high): 16.2.1 makes the NSS pad free under software management, and
 * a select the bus AO drives is the arrangement docs/design/spi-bus.md
 * gives every target.
 *
 * Two of the chapter's sentences are slips and this file reads past
 * them: DFF's two codes are both described as "16-bit" (0 is 8-bit,
 * the F1 lineage and the datasheet's "8-bit or 16-bit"), and 16.2.1's
 * description of CPHA says "the CPHA setting" for both values (1
 * samples on the second edge, 0 on the first - table 16-1, and the four
 * figures that follow). The DMA requests are channels 2 (receive) and
 * 3 (transmit), table 8-2: an engine slot naming any other channel is
 * refused at compile time, because on this family the channel is the
 * request and a wrong one moves nothing.
 *
 * The disable procedure (16.2.7 for MODF, and the F1 lineage's rule
 * this chapter does not spell): clearing SPE while a frame is in
 * flight truncates it on the wire. `Spi::disable()` waits for TXE (the
 * buffer moved into the shifter) and then for BSY to fall (the shifter
 * idle) before it drops the bit, each wait bounded, and reports whether
 * a wait ran out; it does not wait for RXNE. Clearing MODF is a STATR
 * read followed by a CTLR1 write; clearing OVR a DATAR read followed by
 * a STATR read - two sequences the resource keeps.
 *
 * Where the polled loops run from: the host's placement, its last template
 * parameter (util/placement.hpp). The code flash answers behind two wait
 * states at 48 MHz (RM 18.3.1), and every taken branch of a loop is a
 * fetch the core waits for there. Under `InFlash`, the default, the two
 * loops are inline in the phase that runs them, as if there were no
 * trampoline; under `InRam` each width of each is ONE out-of-line
 * instance in `.fast_text.brio`, which ld/ collects into .data and the crt
 * copies to SRAM, flattened (no call left on the path), a call a phase.
 * The receive loop's dead bus a frame then falls from 25 cycles to 11 at
 * HCLK/4, and the transmit-only loop holds the wire at HCLK/2 (measured,
 * docs/ch32v00x/spi.md); the cost constant the engines' polled threshold
 * is drawn from follows the placement. The pump is not placed: inline in
 * the vector under either tag, because a call into SRAM from a vector in
 * the flash and the return cost its short body more than SRAM saves it
 * (measured: 176 cycles an entry against 167 inline); a program that wants
 * it in SRAM places the vector (`.fast_text`), 175 cycles a frame at
 * HCLK/4 against 259.
 *
 * The pads come with their remap code (afio.hpp, table 7-12):
 * spi1_pins_for(code) is a whole column, and init() writes the code.
 *
 * Not covered yet (docs/ch32v00x/spi.md's two lists, with their
 * reasons): the simplex modes (BIDIMODE/BIDIOE, RXONLY) beyond the
 * resource's configuration bits - no user; the hardware NSS arrangements
 * as the engine's select (the resource has them, the engine's select is
 * a GPIO on purpose); the CRC beyond the resource's verbs - born with a
 * device that checks one; the CH32V003F4P6's client in modes 1 and 2,
 * which nothing refuses on that part because nobody measured it. Not
 * bench-verified: HSCR's high-speed read mode, whose rate formula
 * (16.3.1's note: HCLK/(BR+2)) wants a scope on SCK; the client's runs,
 * the suite and the pump's OVR test on the CH32V003F4P6; the data both
 * ways above 6 MHz, the pair's ceiling being the far client's; and the
 * CH32V003's default hold-off against that part's Sleep, which the pump
 * with two frames in flight overruns while the loop idles between frames.
 */

#pragma once


#include <stdint.h>

#include <cstddef>
#include <optional>
#include <span>
#include <type_traits>

#include "ch32v00x/afio.hpp"
#include "ch32v00x/delay.hpp"
#include "ch32v00x/device.hpp"
#include "ch32v00x/dma_engine.hpp"
#include "ch32v00x/pfic.hpp"
#include "ch32v00x/pin.hpp"
#include "ch32v00x/platform.hpp"
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

    /// The drain-then-disable procedure: the buffer empty (TXE), then
    /// the shifter idle (not BSY), then SPE down; RXNE is not waited for.
    /// Each wait is bounded; false says one ran out and the bit was
    /// dropped regardless.
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

    /// The raised sources that are enabled, the answer every resource's
    /// ISR body in brio gives: RXNE and TXE under their own enables, the
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
// The two other codes this host answers are the vocabulary's
// (util/spi_bus.hpp). spi_overrun: the pump lost a frame - the receive
// buffer overran (16.2.7) because the handler read frame k later than
// one frame time after it came back; the transaction still ran to its
// end, every frame went OUT, one that came back is missing from the in
// buffer - the witness of a write-ahead threshold that was too low for
// the image it ran in. spi_stalled: a polled frame's flag never came
// within the transaction's budget - a clock that stopped, a block that
// is dead; the select is released.

/// A frame's time on the wire in HCLK cycles: its bits at SCK = HCLK over
/// two to the code plus one (16.3.1's BR table).
constexpr uint32_t spi_frame_cycles(SpiClock c, SpiDataSize bits) {
    return static_cast<uint32_t>(spi_data_bits(bits)) << (static_cast<uint8_t>(c) + 1u);
}

/*
 * The write-ahead threshold, and the arithmetic behind it.
 *
 * The block has one transmit buffer, one shift register and one receive
 * buffer (figure 16-1): a frame written while another shifts waits in
 * the buffer and moves into the shifter the instant the one before it
 * ends, with no gap on SCK - "if the TXE flag is set, fill the data
 * register, maintain the complete data flow" (16.2.2). So a pump that
 * keeps two frames in flight (frame k + 1 shifting while the handler for
 * frame k runs) keeps the bus busy through the handler; one that keeps
 * one leaves the bus idle from RXNE to the handler's next write.
 *
 * Two in flight has a price: the receive buffer is one frame deep. RXNE
 * for frame k rises at frame k's last sampling edge, and frame k + 1
 * ends one frame time later - if the handler has not read DATAR by then,
 * OVR (16.2.7: "there is unread data in the receive buffer"), frame
 * k + 1's answer lost. The handler reads frame k within
 *
 *     the longest window the SPI's service waits behind
 *   + the core's interrupt entry
 *   + the handler's own instructions from its vector to the DATAR read
 *
 * cycles after RXNE, and two in flight is safe only where the frame time
 * exceeds that sum. On this core no interrupt nests over another
 * (docs/design/kernel.md section 1, "The model in one page", and the
 * platform's promise): the SPI's handler waits behind every other
 * handler's whole body as it waits behind a critical section, so the
 * first term is the longer of the image's longest handler and its longest
 * masked window. Counted in the release listings of bench_ch32 and
 * test_ch32_spi (WCH gcc 15.2, -Os, a cost model of 2.5 cycles a
 * straight-line instruction, HPE entry 29 and exit 25 measured in
 * test_ch32_platform):
 * the console transport's receive path with its RxActivity post, ~75
 * instructions behind the hardware prologue and epilogue, ~245 cycles;
 * the kernel's post() of a 48-byte bus request under the mask, the copy
 * three word turns of it, ~160; the USART handler of the bench app with
 * its two stamps ~300. That last figure rounded up is the default of
 * the host's `hold_off_cycles` parameter, and only a default: the first
 * term is a fact of the image, not of the family, so an image whose
 * handlers or masked windows are longer declares its own (and one that
 * knows its own are shorter may declare that), and an spi_overrun in a
 * TransferDone is the witness of a figure declared too short. The second and
 * third terms are the hardware prologue's 29 cycles and the seven
 * instructions from the vector to the DATAR read in bench_ch32's
 * spi1_handler (the app's own test of which host the vector serves,
 * then lui, lhu STATR, andi, beqz, lhu DATAR), two of them APB loads;
 * the OVR test is a second STATR load after the DATAR one (isr()).
 *
 * At 48 MHz an 8-bit frame is 16 cycles at /2 and doubles per code: under
 * the default the sum of 368 is first exceeded at /64 (512 cycles) for
 * 8-bit frames and at /32 (512) for 16-bit ones; a hold-off of 600
 * moves both one code slower. Below those rates the pump keeps one
 * frame in flight, as the polled receive loop does at every rate - the
 * loop in main context waits behind the same handlers, and a lost frame
 * on a read is a wrong answer where an idle bus is only a slower one.
 * The polled WRITE needs no threshold at all: it never reads the answers
 * (below).
 */
/// The hold-off of the images counted above: the default of SpiHost's
/// `hold_off_cycles`.
inline constexpr uint32_t spi_default_hold_off_cycles = 320;
inline constexpr uint32_t spi_pump_read_cycles = 29u + 7u * 2u + 5u;   // HPE entry + seven instructions, two of them APB loads

/// The frame, in HCLK cycles, that a read can come after RXNE at the
/// latest under a hold-off: a longer one keeps two in flight.
constexpr uint32_t spi_write_ahead_min_frame_cycles(uint32_t hold_off_cycles) {
    return hold_off_cycles + spi_pump_read_cycles;
}

/// Two frames in flight at this code and width, under this hold-off?
constexpr bool spi_write_ahead_safe(SpiClock c, SpiDataSize bits, uint32_t hold_off_cycles) {
    return spi_frame_cycles(c, bits) > spi_write_ahead_min_frame_cycles(hold_off_cycles);
}
/// The fastest code at which the pump keeps two frames in flight for a
/// width under a hold-off (every slower code does too).
constexpr SpiClock spi_write_ahead_from(SpiDataSize bits, uint32_t hold_off_cycles) {
    for (uint8_t code = 0; code < 8u; ++code) {
        if (spi_write_ahead_safe(static_cast<SpiClock>(code), bits, hold_off_cycles)) {
            return static_cast<SpiClock>(code);
        }
    }
    return SpiClock::div256;
}

/**
 * SpiHost<n, pins, TxEngine, RxEngine, hold_off_cycles, Placement>
 *
 * The engine SpiBus (util/spi_bus.hpp = BusMaster, the arbiter) drives.
 * Its Request carries the field names SpiHost's Request has on every
 * stratum: the bus AO asserts the request's own chip select around the
 * transaction, a command phase goes out with D/C low and a data phase
 * with it high, the data phase has an optional out buffer (null = 0xFF
 * dummies) and an optional in buffer (null = the frames read back are
 * discarded), and every buffer is lent until the reply lands: the client
 * neither reads nor writes it from the post to its SpiDone. The mode,
 * rate and frame size travel per request, so a shared bus serves
 * devices that disagree on them; the bit order is a property of the
 * wire and a bus-level verb.
 *
 * The request is lent for the call (util/bus_master.hpp): a polled
 * request completes inside start() and is read through the reference
 * and copied nowhere; an asynchronous one has what its tenure (the span
 * from start() to the completion) needs -
 * the two pins, the three buffers, the two lengths, the width, the
 * completion style - copied as word stores into the host's own tenure
 * record, never through a call. The Request's fields are laid out so
 * that those words are the first nine and nothing in it is padding.
 *
 * Two polled loops, from 16.2.2. A phase whose answers nobody wants (the
 * command phase, a write with no in buffer) runs the transmit-only loop:
 * a frame is written whenever TXE says the buffer is free, which it is a
 * whole frame time before the shifter needs it, so the bus never idles
 * and an interrupt that delays the loop only pauses the stream; the
 * answers pile into the receive buffer and overrun it, which costs
 * nothing (16.2.7: the buffer keeps the old frame, the transmit side is
 * untouched) and is cleared at the phase's end by the DATAR-then-STATR
 * read the chapter prescribes, after TXE and then BSY have said the last
 * frame is out. A phase with an in buffer runs the receive loop: one
 * frame in flight - write k, wait RXNE, read k and write k + 1 in the
 * next instruction - because two in flight would lose a frame to any
 * interrupt longer than a frame time (the threshold above), and a lost
 * answer is wrong where an idle bus is slow. The next frame's load and
 * the store of the one just read sit inside the wire time; the bus's
 * dead time per frame is the poll's last turn, the DATAR read and the
 * DATAR write. Both loops are specialized by frame width and both spend
 * one bounded budget for the whole transaction, not one per frame.
 *
 * The pump runs on RXNE: the frame that came back is the one witness the
 * bus is idle for the next, so TXE (a frame early) is never the pump's
 * edge. Per frame one interrupt - this silicon has no FIFO - and in the
 * handler the DATAR read is the fifth instruction of the body, the
 * STATR load that tests OVR the next, and the next frame's write,
 * prepared by the previous handler inside the wire time, the ninth after
 * the read; the store of the frame read and the walk to the frame after
 * come behind the write. Above the threshold
 * (the hold-off below decides it) the handler keeps two frames in flight
 * (the phase primed with two, the handler for frame k writing k + 2),
 * below it one. An OVR seen in the STATR read that follows the DATAR
 * read gives the transaction spi_overrun, the lost frame counted so the
 * phase still ends.
 *
 * The engine slots: `DmaTxEngine<3, Elem>` and `DmaRxEngine<2, Elem>`,
 * both or neither, on the two channels table 8-2 wires to this instance
 * and no others. The data phase is full duplex and its completion is
 * the receive block's: every frame that came back was clocked out
 * first, so the transmit block has ended when the receive block ends,
 * and the transmit engine is armed for its errors alone - one interrupt
 * a transaction, the receive channel's. The beat (the width of one DMA
 * access) is the frame: engines
 * of `uint16_t` (DATAR's width) move a 16-bit request in half-word
 * beats, the buffer's two bytes low-first being one half-word in
 * memory, when both of its buffers sit on a half-word boundary - 8.3.6:
 * a 16-bit access ignores the address's bit 0, so an odd buffer would
 * move the wrong bytes and goes to the pump instead; engines of
 * `uint8_t` serve the 8-bit requests alone. A data phase shorter than
 * `dma_min_frames` takes the pump even with engines bound, and a POLLED
 * request takes the engines only with an in buffer and from
 * `dma_min_frames_polled` on - the transmit-only loop runs at the wire
 * and the engines' fixed cost buys it nothing (the two constants and
 * their inputs below).
 *
 * Frames in a byte buffer: one byte per 8-bit frame, two bytes low-first
 * per 16-bit frame - the same rule for every SPI host of brio.
 *
 * The hold-off is the image's: `hold_off_cycles` is the longest, in core
 * cycles at the clock the host runs at, that this image keeps the host's
 * vector from running: the longer of its longest handler (no interrupt
 * nests over another) and its longest masked window. With the pump's own
 * path to the DATAR read it is the write-ahead threshold (the arithmetic
 * above the class); its default, spi_default_hold_off_cycles, is the
 * longest handler counted in this family's own bench and suite images.
 * An image measures its own handlers with util/bench.hpp's IsrMeter, a
 * stamp pair at each vector's first and last statement - `isr` over
 * `irq` is a handler's body, the hardware prologue's entry and exit
 * (docs/ch32v00x/platform.md) on top; bench_ch32 meters the tick
 * (letter t - a suite's letter is one command key of its console and
 * the group of verdicts or measurements it runs), the console (p), the
 * DMA's vectors (d, s) and this pump (e) that way - and an spi_overrun
 * in a TransferDone says the figure it declared was too short. On the
 * CH32V003 the default does not count that part's Sleep: an interrupt
 * that finds the V2A freshly asleep is served up to some 3150 cycles
 * late (docs/ch32v00x/platform.md), and the pump with two frames in
 * flight overruns when the loop idles between frames (measured at
 * HCLK/64, docs/ch32v00x/spi.md; listed there as not yet answered).
 */
template <uint8_t n, SpiPins pins = spi1_default_pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine, uint32_t hold_off_cycles = spi_default_hold_off_cycles,
          typename Placement = InFlash>
class SpiHost {
    using S = Spi<n>;

    static_assert(HotPlacement<Placement, Ch32v00xPlatform<>>,
                  "brio SpiHost: the placement is util/placement.hpp's InFlash (the default) or InRam");

    static_assert(hold_off_cycles > 0u,
                  "brio SpiHost: a hold-off of zero cycles is no image's - an image with no "
                  "handler of its own still has the tick's, and every critical section is one");

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from ch32v00x/dma.hpp, or NoDmaEngine (the default)");
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

    /// The transaction descriptor. THE FIELD NAMES ARE THE CONTRACT (the
    /// other strata's, filled by name); the ORDER is this host's: the
    /// eight words an asynchronous tenure copies come first, then ONE
    /// word holding the three fields apply() compares and the setup
    /// time, then the reply - no padding anywhere.
    struct Request {
        PinRef cs;   ///< asserted low around the transaction
        PinRef dc;   ///< display D/C line; null = no such pin
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len;      ///< phase 2 length, in FRAMES
        uint8_t cmd_len;   ///< in FRAMES (see the class comment)
        /// Completion style: false = per-frame ISR pump, true = POLLED
        /// inside start() (see the class comment).
        bool polled = false;
        /// Microseconds between the CS assertion and the first clock -
        /// what a device's datasheet calls CS setup. Spent spinning in
        /// start(), main context; 0 = none.
        uint8_t cs_setup_us = 0;
        /// Per-transaction bus configuration: a shared bus's devices
        /// each name their own, and the engine reprograms the peripheral
        /// only when something CHANGED.
        SpiClock clock = SpiClock::div16;
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;
        ReplyTo<SpiDone> reply;
    };
    static_assert(std::is_trivially_copyable_v<Request>);
    static_assert(sizeof(Request) == 40u, "the Request is forty bytes with no padding on this ABI");
    static_assert(offsetof(Request, cmd) == 16u && offsetof(Request, rx) == 24u && offsetof(Request, len) == 28u &&
                  offsetof(Request, polled) == 31u && offsetof(Request, cs_setup_us) == 32u &&
                  offsetof(Request, clock) == 33u && offsetof(Request, mode) == 34u && offsetof(Request, bits) == 35u &&
                  offsetof(Request, reply) == 36u,
                  "the tenure's words are the first eight of the Request, the configuration the ninth");

    /// THE PUMP AGAINST THE ENGINES, two costs measured in ONE session of
    /// bench_ch32 on the CH32V006K8 (letters s and e, three runs each,
    /// docs/ch32v00x/spi.md). The engines' fixed cost per transaction -
    /// the launch, the receive channel's completion vector with
    /// finish_dma() and the select's release: letter s's 864 cycles above
    /// the wire, less the instrument that line carries (the request built
    /// inside the stopwatch, BenchIdle, the stopwatch, the handler's stamp
    /// pair: 360) - 504. The pump's cost per frame: the handler's metered
    /// body for an 8-bit frame with an in buffer, letter e's 256-frame
    /// write - 177 (176.3 to 176.7). Neither is placed (the file header):
    /// both run from the flash under either placement. A data phase of
    /// fewer frames than their quotient costs less on the pump.
    static constexpr uint32_t engine_fixed_cycles = 504;
    static constexpr uint32_t pump_frame_cycles = 177;
    static constexpr uint16_t dma_min_frames =
        static_cast<uint16_t>((engine_fixed_cycles + pump_frame_cycles - 1u) / pump_frame_cycles);
    /// Are the polled loops placed in SRAM (the file header)?
    static constexpr bool placed_in_ram = std::is_same_v<Placement, InRam>;
    /// The receive loop's dead bus per frame - the poll's last turn, the
    /// DATAR read, the DATAR write - at HCLK/4, against the wire's 32: 25
    /// cycles from the flash (56.7 a frame), 11 from SRAM (42.8), each in
    /// its placement - against the same fixed cost: a polled request with
    /// an in buffer rides the engines from this many frames on. The
    /// transmit-only loop never does: it runs at the wire.
    static constexpr uint32_t polled_gap_cycles = placed_in_ram ? 11u : 25u;
    static constexpr uint16_t dma_min_frames_polled = static_cast<uint16_t>(engine_fixed_cycles / polled_gap_cycles);
    static_assert(dma_min_frames == 3u && dma_min_frames_polled == (placed_in_ram ? 45u : 20u),
                  "the two thresholds the measured costs give, per placement (docs/ch32v00x/spi.md)");

    // ---- lifecycle ----------------------------------------------------------

    /// Bring the instance up as a host: gate, reset, configuration,
    /// pads, the PFIC line. `clock` is the app's Clock tag, the one
    /// truth of HCLK (this family has no APB prescaler, so HCLK is what
    /// BR divides). `max_sck_hz` is an optional CEILING for the whole
    /// bus, re-resolved on rebase(); 0 = none. False when the ceiling
    /// cannot be produced at this clock, or the boot configuration is
    /// refused. Call it in main, after the clock task's init() and before
    /// the kernel's run(); with engines it arms both (receive at
    /// very_high, transmit at high, errors alone). It leaves the NSS pad
    /// and every chip select untouched: a request's select is a GPIO the
    /// program sets up as an output, high, before the first request.
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
            // The transmit block ends before the receive block does, which
            // is the one interrupt a transaction takes: its errors alone.
            // The receive side at VERY_HIGH, the transmit at HIGH
            // (DmaPriority's rule): DATAR's receive half holds one frame,
            // and a receive channel the transmit one outran would find the
            // next frame on top of its last (OVR, 16.2.7); a transmit channel that
            // waits only stops the host's clock between two frames.
            TxEngine::arm(S::data_address(), TxEngine::flag_error, DmaPriority::high);
            RxEngine::arm(S::data_address(), DmaPriority::very_high);
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
        enable_applied();
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
        // A slower rate is a LARGER code, so the ceiling clamps codes from
        // below; code 0 (HCLK/2) as the floor clamps nothing, which is
        // also what "no ceiling" means.
        floor_code_ = ceiling_ ? static_cast<uint8_t>(*ceiling_) : 0u;
        cs_rate_ = delay_rate(hz);
        // A new ceiling may clamp the last request's clock to another
        // code: the next request must be folded and compared again.
        request_key_ = 0xFFFF'FFFFu;
    }

    /// The BR code that produces at most `hz` of SCK at the HCLK last
    /// seen - the chooser a device's datasheet limit is spoken to.
    static std::optional<SpiClock> clock_for(uint32_t hz) { return spi_rate_for(hclk_hz_, hz); }
    /// What a request at this code really runs at, ceiling included.
    static uint32_t sck_hz(SpiClock c) { return spi_sck_hz(hclk_hz_, clamp(c)); }
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    static uint32_t reference_hz() { return hclk_hz_; }

    /// The fastest code at which a receive of that width runs with two
    /// frames in flight under this host's hold-off (the class comment);
    /// every slower code does too. A frame is HCLK cycles, so the answer
    /// is the same at every clock.
    static constexpr SpiClock write_ahead_from(SpiDataSize bits) {
        return spi_write_ahead_from(bits, hold_off_cycles);
    }

    /// Put the peripheral at this mode, rate and frame size NOW, moving
    /// no data - for callers that frame the select window themselves
    /// (Request.cs null). A mode change is a CPOL FLIP ON THE WIRE and a
    /// selected client counts it as an edge: prime FIRST, then select.
    static void prime(SpiMode m, SpiClock c, SpiDataSize bits = SpiDataSize::bits8) {
        Request r{};
        r.clock = c;
        r.mode = m;
        r.bits = bits;
        apply(r, clamp(c));
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
        enable_applied();
        return ok;
    }
    static bool lsb_first() { return applied_.lsb_first; }

    // ---- the transfer -------------------------------------------------------

    /// Begin a transaction (SpiBus calls it from main context; a program
    /// posts the Request to the arbiter instead). True when it completed
    /// synchronously (polled requests, the empty one); false when it runs
    /// on the ISR and a TransferDone follows - exactly util/bus_master.hpp's
    /// engine contract. A polled request waits here: cs_setup_us spun,
    /// then every frame's flag under one bounded budget (spi_stalled in
    /// status() when it runs out, the select released) or, on the
    /// engines, the block's end under a bounded spin (spi_dma_fault).
    /// Masks nothing. The buffers stay lent until the arbiter's reply.
    static bool start(const Request& r) {
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
        // No flush of the receive buffer here: every path below leaves it
        // empty at its end (the receive loop and the pump read every
        // frame, the transmit-only loop flushes after its tail, the
        // receive engine takes every frame and the fault exits flush),
        // and init() and recover() flush what a reset or a prime left.

        if (r.polled) {
            uint32_t budget = poll_budget(code, r.bits, static_cast<uint32_t>(r.cmd_len) + r.len);
            bool ok = true;
            if constexpr (has_engines) {
                if (r.rx.get() != nullptr && r.len >= dma_min_frames_polled && dma_serves(r)) {
                    // The command phase on the transmit-only loop, then
                    // the engines over the data phase, spun to their end.
                    if (r.cmd_len != 0u) {
                        ok = poll_write_phase(r.bits, r.cmd.get(), r.cmd_len, budget);
                    }
                    r.dc.set();
                    if (ok) {
                        keep(r);
                        // The beat is this request's frame: launch_dma()
                        // reads the step, which the asynchronous path
                        // below sets too - a polled 16-bit request after
                        // an 8-bit asynchronous one launched byte beats
                        // before it was set here (test_ch32_spi letter d).
                        step_ = spi_frame_is_halfword(r.bits) ? 2u : 1u;
                        launch_dma();
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
                    ok = poll_duplex_phase(r.bits, r.tx.get(), r.rx.get(), r.len, budget);
                }
            }
            r.cs.set();
            if (!ok) {
                stalled();
            }
            return true;
        }

        // The asynchronous paths: what the tenure needs, copied.
        keep(r);
        step_ = spi_frame_is_halfword(r.bits) ? 2u : 1u;
        ahead_ = spi_write_ahead_safe(code, r.bits, hold_off_cycles) ? 2u : 1u;
        if constexpr (has_engines) {
            dma_tenure_ = r.len >= dma_min_frames && dma_serves(r);
            if (dma_tenure_ && r.cmd_len == 0u) {
                launch_dma();
                return false;   // dma_isr() is the completion edge
            }
        }
        in_cmd_ = r.cmd_len != 0u;
        // The first frames go in BEFORE the interrupt is armed: the flag
        // stands until the handler reads DATAR, and a handler let in
        // between the phase's first write and the store of its counts
        // sees a count of zero, writes nothing, and the phase never ends
        // - measured at HCLK/4, where the first frame is back seventeen
        // instructions before begin_phase() has stored left_.
        if (in_cmd_) {
            begin_phase(r.cmd.get(), nullptr, r.cmd_len);   // the echo discarded
        } else {
            begin_phase(r.tx.get(), r.rx.get(), r.len);
        }
        S::rxne_interrupt(true);
        return false;   // isr() pumps the rest
    }

    /// The instance's interrupt body - call from its vector (the
    /// program's spi1_handler) and nowhere else. Only RXNE is ever armed
    /// by this engine, and reading DATAR is both the capture and the
    /// acknowledgement. True when the transaction just completed (CS
    /// released): the edge the app's glue posts TransferDone on, with
    /// status() as its payload.
    ///
    /// OVR is read after DATAR, from a second STATR load: with two frames
    /// in flight the one behind frame k may complete at any moment before
    /// the DATAR read, and a status copy taken before it would miss the
    /// OVR that completion raises - frame k + 1 lost uncounted, and the
    /// phase waiting for a frame that never comes back. The load after
    /// DATAR sees every overrun up to the read, and DATAR-then-STATR is
    /// 16.2.7's clear: the flag goes down in the same handler, never left
    /// standing for the next transaction's first. One APB load a frame.
    ///
    /// Inline in the vector under either placement (the file header): a
    /// call into SRAM from a vector in the flash, and the return into the
    /// flash, cost this short body more than SRAM saves it - measured. A
    /// program that wants the pump in SRAM places the vector.
    [[gnu::always_inline]] static bool isr() {
        if ((S::regs().STATR & spi_rxne) == 0u) {
            return false;
        }
        const uint16_t in = S::regs().DATAR;   // the fifth instruction of the body
        const bool lost = (S::regs().STATR & spi_ovr) != 0u;
        if (lost) {
            status_ = spi_overrun;
        }
        // The prepared frame out: with one in flight this is the bus's
        // restart, with two it keeps the buffer full. Then the frame
        // read stored, and the one after prepared inside the wire time.
        if (left_ != 0u) {
            S::regs().DATAR = next_;
            --left_;
        }
        if (in_ != nullptr) {
            store_in(in);
        }
        if (lost && to_read_ > 1u) {
            // 16.2.7: the frame after this one ended while this one stood
            // unread; the buffer kept this one and lost that one. Its slot
            // is skipped and it is counted as read, so the phase ends.
            --to_read_;
            if (in_ != nullptr) {
                in_ += step_;
            }
        }
        if (left_ != 0u) {
            next_ = next_out();
        }
        if (--to_read_ != 0u) {
            return false;
        }
        // The phase ended: its last frame is back and read.
        if (in_cmd_) {
            in_cmd_ = false;
            tenure_.dc.set();
            if (tenure_.len == 0u) {
                return finish_pump();
            }
            if constexpr (has_engines) {
                if (dma_tenure_) {
                    S::rxne_interrupt(false);
                    launch_dma();
                    return false;
                }
            }
            begin_phase(tenure_.tx, tenure_.rx, tenure_.len);
            return false;
        }
        return finish_pump();
    }

    /// The DMA channels' interrupt body - call from both channels'
    /// vectors (dma1_channel2_handler and dma1_channel3_handler, one
    /// vector per channel here) and nowhere else. The receive channel's
    /// completion ends the transaction - the one interrupt it takes, the
    /// transmit channel being armed for errors alone - and a transfer
    /// error on either ends it with spi_dma_fault. INTFR is read once for
    /// both. Compiles away on an engineless host. True when the
    /// transaction just completed.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
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
                    return finish_dma(spi_ok);
                }
            }
        }
        return false;
    }

    /// The engine's completion status, for the TransferDone payload:
    /// spi_ok, spi_overrun (the pump lost a frame), spi_stalled (a polled
    /// flag never came) or spi_dma_fault (a block the engines could not
    /// finish). Read it in the vector right after isr() or dma_isr()
    /// returned true; start() sets it to spi_ok on entry.
    static uint8_t status() { return status_; }

    /// Put the ENGINE back where start() is legal (the verb a timed
    /// SpiBus calls on a transaction that never answered): the select
    /// released FIRST, the engines put away and re-claimed, the
    /// peripheral reset and reconfigured to the applied state. False
    /// when a bounded wait ran out.
    static bool recover() {
        tenure_.cs.set();
        if constexpr (has_engines) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();   // the binding stands: the next start() needs no arm()
        }
        Pfic::disable(S::irq());
        in_cmd_ = false;
        to_read_ = 0;
        left_ = 0;
        dma_active_ = false;
        dma_done_ = false;
        S::rxne_interrupt(false);
        const bool ok = S::disable();
        S::reset();
        const bool cfg = S::configure(applied_);
        enable_applied();
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
    /// What the asynchronous paths read after start() has returned: the
    /// request's tenure, its first eight words, copied by keep() as word
    /// loads and stores.
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

    /// The frame at `p` in the strata's byte order, and its store.
    template <typename T>
    [[gnu::always_inline]] static uint16_t load_frame(const uint8_t* p) {
        if constexpr (sizeof(T) == 2u) {
            return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
        } else {
            return p[0];
        }
    }
    template <typename T>
    [[gnu::always_inline]] static void store_frame(uint8_t* p, uint16_t v) {
        p[0] = static_cast<uint8_t>(v);
        if constexpr (sizeof(T) == 2u) {
            p[1] = static_cast<uint8_t>(v >> 8);
        }
    }

    /// ONE budget of poll turns for the whole transaction: a frame's
    /// cycles per frame, two frames more for the priming and the drain.
    /// A turn is at least three instructions, so the budget is several
    /// times the turns the wire takes, and a shift and no multiply (the
    /// CH32V003 has none). Spent across the phases; zero when a flag
    /// never came.
    [[gnu::always_inline]] static uint32_t poll_budget(SpiClock code, SpiDataSize bits, uint32_t frames) {
        const uint32_t log2_frame = static_cast<uint32_t>(code) + 4u + (spi_frame_is_halfword(bits) ? 1u : 0u);
        return (frames + 2u) << log2_frame;
    }

    /// The loops' two widths, each one function of the host's placement
    /// (the file header): inline in the phase under InFlash, the
    /// trampoline's instance for that width's lambda under InRam - a call
    /// a phase, the loop flattened into it. The closure (the buffers, the
    /// count, the budget's address) is built once and passed by address.
    /// NOT INLINED into start(), which calls them from three places: there
    /// both widths of a loop would be copied into it, measured a kilobyte
    /// more of flash on the CH32V003's suite images.
    [[gnu::noinline]] static bool poll_write_phase(SpiDataSize bits, const uint8_t* out, uint16_t count, uint32_t& budget) {
        return spi_frame_is_halfword(bits) ? poll_write_at<uint16_t>(out, count, budget)
                                           : poll_write_at<uint8_t>(out, count, budget);
    }
    [[gnu::noinline]] static bool poll_duplex_phase(SpiDataSize bits, const uint8_t* out, uint8_t* in,
                                                    uint16_t count, uint32_t& budget) {
        return spi_frame_is_halfword(bits) ? poll_duplex_at<uint16_t>(out, in, count, budget)
                                           : poll_duplex_at<uint8_t>(out, in, count, budget);
    }
    template <typename T>
    [[gnu::always_inline]] static bool poll_write_at(const uint8_t* out, uint16_t count, uint32_t& budget) {
        return Hot<Placement>::run([out, count, &budget] { return poll_write<T>(out, count, budget); });
    }
    template <typename T>
    [[gnu::always_inline]] static bool poll_duplex_at(const uint8_t* out, uint8_t* in, uint16_t count,
                                                      uint32_t& budget) {
        return Hot<Placement>::run(
            [out, in, count, &budget] { return poll_duplex<T>(out, in, count, budget); });
    }

    /// THE TRANSMIT-ONLY LOOP (16.2.2): a frame written whenever TXE says
    /// the buffer is free - a frame time ahead of the shifter's need, so
    /// the clock never pauses - and the answers left to overrun the
    /// receive buffer, which 16.2.7 makes harmless for the transmit side.
    /// The tail waits TXE (the last frame into the shifter) and then not
    /// BSY (out of it); the DATAR-then-STATR read clears RXNE and OVR.
    /// A null `out` clocks 0xFF dummies from a fixed cell. `count` >= 1.
    template <typename T>
    [[gnu::always_inline]] static bool poll_write(const uint8_t* out, uint16_t count, uint32_t& budget) {
        const uint8_t* p = out != nullptr ? out : dummy_bytes_;
        const uint32_t step = out != nullptr ? sizeof(T) : 0u;
        SpiRegs& regs = S::regs();
        uint32_t spins = budget;
        uint32_t left = count;   // a word: a half-word counter costs a zero-extension a turn
        do {
            while ((regs.STATR & spi_txe) == 0u) {
                if (--spins == 0u) {
                    budget = 0;
                    return false;
                }
            }
            regs.DATAR = load_frame<T>(p);
            p += step;
        } while (--left != 0u);
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

    /// THE RECEIVE LOOP: one frame in flight. Frame 0 written; then for
    /// each frame the next one's load, the wait on RXNE, the DATAR read
    /// and the DATAR write back to back, the store of the frame read.
    /// The last frame is read after the loop. `count` >= 1.
    template <typename T>
    [[gnu::always_inline]] static bool poll_duplex(const uint8_t* out, uint8_t* in, uint16_t count, uint32_t& budget) {
        const uint8_t* p = out != nullptr ? out : dummy_bytes_;
        const uint32_t step = out != nullptr ? sizeof(T) : 0u;
        SpiRegs& regs = S::regs();
        uint32_t spins = budget;
        uint32_t left = count;
        regs.DATAR = load_frame<T>(p);   // the shifter is idle: frame 0 moves at once
        while (--left != 0u) {
            p += step;
            const uint32_t next = load_frame<T>(p);   // a word: the barrier would zero-extend a half-word
            // The load above belongs INSIDE the wire time: the compiler
            // would sink it past the poll, onto the bus's dead time.
            __asm__ volatile("" : : "r"(next));
            while ((regs.STATR & spi_rxne) == 0u) {
                if (--spins == 0u) {
                    budget = 0;
                    return false;
                }
            }
            const uint16_t got = regs.DATAR;
            regs.DATAR = static_cast<uint16_t>(next);
            store_frame<T>(in, got);
            in += sizeof(T);
        }
        while ((regs.STATR & spi_rxne) == 0u) {
            if (--spins == 0u) {
                budget = 0;
                return false;
            }
        }
        store_frame<T>(in, regs.DATAR);
        budget = spins;
        return true;
    }

    /// A polled flag that never came: the status, and the buffer flushed
    /// so the next transaction does not read this one's frame.
    static void stalled() {
        status_ = spi_stalled;
        S::flush_rx();
    }

    // ---- the pump -----------------------------------------------------------

    /// The next frame to go out in this phase, read from the out pointer
    /// and advancing it by the width, or the dummy when the phase has no
    /// out buffer. Called at a handler's TAIL and at a phase's start, so
    /// that the handler's path to the DATAR write holds a load of the
    /// prepared frame and nothing of the buffer walk.
    [[gnu::always_inline]] static uint16_t next_out() {
        if (out_ == nullptr) {
            return 0xFFFFu;
        }
        uint16_t v = out_[0];
        if (step_ == 2u) {
            v = static_cast<uint16_t>(v | (static_cast<uint16_t>(out_[1]) << 8));
        }
        out_ += step_;
        return v;
    }
    [[gnu::always_inline]] static void store_in(uint16_t v) {
        in_[0] = static_cast<uint8_t>(v);
        if (step_ == 2u) {
            in_[1] = static_cast<uint8_t>(v >> 8);
        }
        in_ += step_;
    }

    /// Open a phase on the pump: its buffers and count, then the priming
    /// - one frame, or two where the threshold allows (the second after
    /// TXE has said the first moved into the shifter, a few cycles on an
    /// idle bus; if it has not within a short bound, one stays in flight
    /// and the handler writes the second) - and the frame after those
    /// prepared for the first handler. `count` >= 1.
    static void begin_phase(const uint8_t* out, uint8_t* in, uint16_t count) {
        out_ = out;
        in_ = in;
        to_read_ = count;
        S::regs().DATAR = next_out();
        uint32_t left = count - 1u;
        if (ahead_ == 2u && left != 0u) {
            for (uint8_t spins = 32; spins != 0u; --spins) {
                if (S::txe()) {
                    S::regs().DATAR = next_out();
                    --left;
                    break;
                }
            }
        }
        if (left != 0u) {
            next_ = next_out();
        }
        left_ = left;
    }

    [[gnu::always_inline]] static bool finish_pump() {
        S::rxne_interrupt(false);
        tenure_.cs.set();
        return true;
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

    /// The engines serve every request in 8-bit frames, and one in 16-bit
    /// frames when they take half-words and both buffers sit on a
    /// half-word boundary (an absent buffer is the dummy cell, aligned).
    static bool dma_serves(const Request& r) {
        if (!spi_frame_is_halfword(r.bits)) {
            return true;
        }
        if constexpr (engines_carry_halfwords) {
            const uint32_t odd = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(r.tx.get())) |
                                 static_cast<uint32_t>(reinterpret_cast<uintptr_t>(r.rx.get()));
            return (odd & 1u) == 0u;
        } else {
            return false;
        }
    }

    static void launch_dma() {
        dma_done_ = false;
        dma_active_ = true;
        // THE REQUESTS ARE ARMED AROUND THE ENGINES AND NOWHERE ELSE. A
        // request that rises while its channel is disabled is LATCHED by
        // the controller and served at the channel's next enable
        // (measured: with RXDMAEN standing, the echo of a command frame
        // pumped by hand - or the stale frame a previous transaction left
        // in DATAR - lands as the block's first byte and the block comes
        // back shifted by one). So RXDMAEN is raised only once the receive
        // channel is enabled, TXDMAEN only once the transmit one is, and
        // finish_dma() drops both.
        // The RECEIVE channel first: its request rises only when a frame
        // has come back, and the transmit side is what starts the clock.
        // The beat is the frame (dma_serves() checked the alignment).
        if constexpr (engines_carry_halfwords) {
            if (step_ == 2u) {
                launch_beats<uint16_t>();
                return;
            }
        }
        launch_beats<uint8_t>();
    }

    template <typename T>
    static void launch_beats() {
        T* const rx = reinterpret_cast<T*>(tenure_.rx);
        const T* const tx = reinterpret_cast<const T*>(tenure_.tx);
        if (rx != nullptr) {
            (void)RxEngine::start(std::span<T>(rx, tenure_.len));
        } else {
            (void)RxEngine::start_discard(reinterpret_cast<T*>(&rx_sink_), tenure_.len);
        }
        S::dma_requests(false, true);
        if (tx != nullptr) {
            (void)TxEngine::start(std::span<const T>(tx, tenure_.len));
        } else {
            (void)TxEngine::start_fixed(reinterpret_cast<const T*>(&tx_dummy_), tenure_.len);
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
            S::flush_rx();   // a frame the stopped channel never took
        } else {
            // The receive block completing is proof the transmit one did:
            // every frame that came back was clocked out first.
            (void)TxEngine::complete();
        }
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

    // ---- the configuration --------------------------------------------------

    /// A slower rate is a LARGER code, so the ceiling clamps from below.
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
        // The DMA requests are NOT part of the applied configuration:
        // launch_dma() raises them around the engines (see there).
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
    }

    /// The CTLR1 bits a request decides: CPHA and CPOL (the mode), BR
    /// (the rate), DFF (the width).
    static constexpr uint16_t request_bits_mask = spi_cpha | spi_cpol | spi_br_mask | spi_dff;
    static constexpr uint16_t request_bits(SpiMode m, SpiClock c, SpiDataSize bits) {
        uint16_t v = static_cast<uint16_t>(static_cast<uint16_t>(c) << 3);
        if (spi_mode_cpha(m)) { v |= spi_cpha; }
        if (spi_mode_cpol(m)) { v |= spi_cpol; }
        if (bits == SpiDataSize::bits16) { v |= spi_dff; }
        return v;
    }

    /// The request's clock, mode and width AS ONE WORD: the three bytes
    /// lie together behind cs_setup_us at a word boundary (the layout
    /// the static_assert above pins), so one load and a shift read them.
    [[gnu::always_inline]] static uint32_t request_key(const Request& r) {
        uint32_t w;
        // The word is at offset 32 of a word-aligned descriptor (the
        // static_assert above); said so, or the copy is a call.
        __builtin_memcpy(&w, __builtin_assume_aligned(&r.cs_setup_us, 4), sizeof w);
        return w >> 8;
    }

    /// Put the peripheral where this request wants it. The compare is
    /// ONE WORD: the request's three fields against the last request's;
    /// equal, nothing moves. Different, the fields are folded into the
    /// control word and compared with the one in force (two requests
    /// under a ceiling may fold to the same word), and a changed word is
    /// written with the disable/enable pair 16.3.1 asks for DFF ("can
    /// only be written when SPE is 0") and the drain before it, so that
    /// no frame is cut on the wire.
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
        (void)S::disable();                                            // the drain, then SPE down
        S::regs().CTLR1 = static_cast<uint16_t>(want & ~spi_spe);      // the new word, SPE still clear
        S::regs().CTLR1 = want;                                        // and up
        ctlr1_ = want;
    }

    // ---- the state ----------------------------------------------------------

    static inline Tenure tenure_{};
    /// The phase on the pump: its out and in pointers walking the
    /// buffers, the frames still to write and still to read (words: a
    /// half-word counter costs a zero-extension per step), the frame
    /// prepared for the next write.
    static inline const uint8_t* out_ = nullptr;
    static inline uint8_t* in_ = nullptr;
    static inline uint32_t left_ = 0;
    static inline uint32_t to_read_ = 0;
    static inline uint16_t next_ = 0;
    static inline uint8_t step_ = 1;    ///< bytes per frame in the buffers
    static inline uint8_t ahead_ = 1;   ///< frames in flight on the pump, 1 or 2
    static inline bool in_cmd_ = false;
    static inline bool dma_tenure_ = false;
    static inline uint8_t status_ = spi_ok;
    static inline volatile bool dma_done_ = false;
    static inline volatile bool dma_active_ = false;
    /// The dummy frame of the polled loops (0xFF or 0xFFFF, read at a
    /// stride of zero), and the discard cell and the dummy cell of the
    /// engines, half-words so that either beat reads or writes them. The
    /// engines' cells are in SRAM: the transmit channel reads its cell
    /// every frame, and from the flash, behind two wait states at 48 MHz
    /// (RM V1.5 18.3.1), that read cost it 1031 thousandths of the wire
    /// at HCLK/2 (measured, test_ch32_spi letter g) where SRAM gives 1000.
    static constexpr uint8_t dummy_bytes_[2] = {0xFFu, 0xFFu};
    static inline uint16_t rx_sink_ = 0;
    static inline uint16_t tx_dummy_ = 0xFFFFu;
    static inline SpiConfig applied_{};
    static inline uint16_t ctlr1_ = 0;     ///< CTLR1 as it stands, SPE included
    static inline uint32_t request_key_ = 0;   ///< the last request's clock, mode and width as one word
    static inline uint8_t floor_code_ = 0;
    static inline uint32_t hclk_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<SpiClock> ceiling_{};
    static inline DelayRate cs_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/**
 * The modes this part's CLIENT is served in. 16.2.3: "if you need to
 * send in slave mode, you need to set CPOL to mode 2 or mode 3" - and
 * measured against a foreign host whose waveforms are the standard's in
 * all four modes (docs/ch32v00x/spi.md), the CH32V006K8U6's client is
 * served in modes 0 and 3 BOTH WAYS and in modes 1 and 2 NEITHER way:
 * what it hears there is every frame one bit early, what it sends a
 * fixed pattern. So a CH32V006 client is refused in modes 1 and 2,
 * listening or answering. The CH32V003F4P6 shares the block and has not
 * answered on its own die: it refuses nothing here (its gap list).
 */
constexpr bool spi_client_mode_served(SpiMode m) {
    if constexpr (device::part == Ch32Part::v006) {
        return m == SpiMode::mode0 || m == SpiMode::mode3;
    } else {
        return true;
    }
}

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
 * frame late under a host that clocks back to back - the runs below do
 * all of it in one call. A client that falls behind does not stall the
 * bus: what the host reads is whatever the shifter held (16.2.3 makes
 * no underrun flag), a wrong answer and never a missing one.
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

    /// Is this configuration one the client is served in on this part
    /// (spi_client_mode_served)?
    static constexpr bool config_served(const Config& c) { return spi_client_mode_served(c.mode); }

    /// Bring the instance up as a client. The bus clock is required
    /// even though the shifter runs on the host's SCK. False when the
    /// configuration is refused - by the chapter, or by what this part
    /// measured as a client (config_served).
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        if (!config_served(cfg)) {
            return false;
        }
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

    /// The same with the configuration known at compile time: a mode
    /// this part's client is not served in does not compile.
    template <Config cfg, typename Clock>
    static bool init(Clock clock) {
        static_assert(config_served(cfg),
                      "brio SpiClient: the CH32V006's client is served in modes 0 and 3 alone - in modes 1 "
                      "and 2 it hears every frame one bit early and sends a fixed pattern (measured, "
                      "spi_client_mode_served)");
        return init(clock, cfg);
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
    static void enable(uint16_t first) {
        S::enable();
        write(first);
    }
    /// SPE up with the transmit buffer EMPTY - for a listener, or for a
    /// program that loads its first answer later.
    static void enable() { S::enable(); }
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

    // ---- the polled runs ---------------------------------------------------
    //
    // A client's unit is the RUN: the answers to a transaction, or what it
    // brings, served polled in one call - the per-frame verbs above its
    // degenerate case. Two sentences of 16.2.3 shape every one of them. The
    // transmit buffer moves into the shifter "when the first slave receiving
    // sampling edge appears in the SCK", and TXE rises then: answer k + 1
    // may be written from frame k's first sampling edge to frame k + 1's,
    // one frame of window. RXNE rises "after the last clock sampling edge",
    // and the receive buffer is one deep (16.2.7): frame k must be read
    // before frame k + 1 ends. THE EXCHANGE READS IN THE MIDDLE OF A FRAME:
    // on the TXE of frame k it writes answer k + 1 and then reads frame
    // k - 1 on its RXNE, which stands by then - never at the frame's end,
    // where a DATAR read in the bit before the next answer moved into the
    // shifter lost that answer (measured, docs/ch32v00x/spi.md). THE SELECT
    // PAD IS THE WINDOW: a run waits for the pad to FALL and takes a high
    // pad for the close, read once for every four polls of a flag; without
    // an NSS pad a run ends on its count or its budget. Each run writes its
    // first answer (when it has one) and then raises SPE - a no-op when SPE
    // stands -, the order this client's serves were measured exact in. They
    // are ALWAYS INLINE: the program places the function that calls them,
    // and from the flash, behind its two wait states at 48 MHz (RM 18.3.1),
    // the fetch is what bounds them (the document's cost section).

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
     * ANSWER a transaction with `out`, polled on TXE alone, reading
     * nothing. Call it with the buffer EMPTY and the select still HIGH:
     * the first answer goes into the buffer at once, every later one on
     * the TXE of the frame before it. Ends when the host raises the
     * select; the frames past the run's end carry what the shifter is left
     * with. The receive buffer overruns from the second frame (nothing is
     * read, harmless to the transmit side by 16.2.7) and the tail clears
     * it with the chapter's DATAR-then-STATR read.
     */
    template <typename T>
    [[gnu::always_inline]] static Run answer(std::span<const T> out, uint32_t spins) {
        static_assert(run_frame<T>, "brio SpiClient: a frame of this block is eight bits or sixteen (DFF): "
                                    "a run is over uint8_t or uint16_t");
        static_assert(pins.miso.valid(), "brio SpiClient: an answering run needs a MISO pad to answer on");
        SpiRegs& regs = S::regs();
        const T* const first = out.data();
        const T* const end = first + out.size();
        const T* p = first;
        RunEnd how = RunEnd::closed;
        if (p != end) {
            regs.DATAR = *p++;
        }
        raise_spe();
        if (!await_select(spins)) {
            how = RunEnd::stalled;
            goto tail;
        }
        if (p != end) {
            do {
                const T next = *p;
                __asm__ volatile("" : : "r"(next));   // loaded before the poll, written right after it
                if (!await<spi_txe>(spins, how)) {
                    goto tail;
                }
                regs.DATAR = next;
            } while (++p != end);
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
        (void)regs.DATAR;   // 16.2.7's clear: DATAR, then STATR
        (void)regs.STATR;
        return {static_cast<uint16_t>(p - first), how};
    }

    /**
     * EXCHANGE: answer with `out` and keep what the host sends in `in`, the
     * run's length the shorter of the two. On the TXE of frame k the
     * answer k + 1 is written, and then frame k - 1 - complete since before
     * frame k began - is read on its RXNE, one poll that finds it; the
     * last frame is read on its own RXNE. Ends `complete` when the last
     * frame is in, `closed` when the host raised the select first (what
     * arrived before is in `in`), `stalled` on the budget. The receive side
     * is cleared at the start, the select high.
     */
    template <typename T>
    [[gnu::always_inline]] static Run exchange(std::span<const T> out, std::span<T> in, uint32_t spins) {
        static_assert(run_frame<T>, "brio SpiClient: a frame of this block is eight bits or sixteen (DFF): "
                                    "a run is over uint8_t or uint16_t");
        static_assert(pins.miso.valid(), "brio SpiClient: an answering run needs a MISO pad to answer on");
        const uint32_t len = out.size() < in.size() ? out.size() : in.size();
        if (len == 0u) {
            return {0, RunEnd::complete};
        }
        SpiRegs& regs = S::regs();
        (void)regs.DATAR;   // the receive side cleared: DATAR, then STATR (16.2.7)
        (void)regs.STATR;
        const T* p = out.data();
        const T* const p_end = p + len;
        T* const first = in.data();
        T* const q_end = first + len;
        T* q = first;
        RunEnd how = RunEnd::complete;
        regs.DATAR = *p++;   // answer 0, before the select falls
        raise_spe();
        if (!await_select(spins)) {
            return {0, RunEnd::stalled};
        }
        if (p != p_end) {
            // Frame 0 began: answer 1 in, nothing to read yet.
            const T second = *p;
            if (!await<spi_txe>(spins, how)) {
                goto drain;
            }
            regs.DATAR = second;
            // Frame k began (TXE): answer k + 1 in, then frame k - 1 - in
            // since before frame k began - read on its RXNE. The answer is
            // loaded BEFORE the poll, so the write follows the read that
            // sees TXE with nothing between them.
            if (++p != p_end) {
                do {
                    const T next = *p;
                    __asm__ volatile("" : : "r"(next));
                    if (!await<spi_txe>(spins, how)) {
                        goto drain;
                    }
                    regs.DATAR = next;
                    // Frame k - 1 ended before frame k began: RXNE stands,
                    // and one poll finds it.
                    if ((regs.STATR & spi_rxne) == 0u && !await<spi_rxne>(spins, how)) {
                        goto drain;
                    }
                    *q++ = static_cast<T>(regs.DATAR);
                } while (++p != p_end);
            }
            // The last frame began: the one before it is in.
            if (!await<spi_txe>(spins, how) || !await<spi_rxne>(spins, how)) {
                goto drain;
            }
            *q++ = static_cast<T>(regs.DATAR);
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
     * when the run began is the run's first; SPE is raised if it is not up.
     * Ends as exchange() does.
     */
    template <typename T>
    [[gnu::always_inline]] static Run receive(std::span<T> in, uint32_t spins) {
        static_assert(run_frame<T>, "brio SpiClient: a frame of this block is eight bits or sixteen (DFF): "
                                    "a run is over uint8_t or uint16_t");
        SpiRegs& regs = S::regs();
        T* const first = in.data();
        T* const q_end = first + in.size();
        T* q = first;
        RunEnd how = RunEnd::complete;
        raise_spe();
        if ((regs.STATR & spi_rxne) == 0u && !await_select(spins)) {
            return {0, RunEnd::stalled};
        }
        if (q != q_end) {
            do {
                if (!await<spi_rxne>(spins, how)) {
                    if ((regs.STATR & spi_rxne) != 0u) {
                        *q++ = static_cast<T>(regs.DATAR);
                    }
                    break;
                }
                *q = static_cast<T>(regs.DATAR);
            } while (++q != q_end);
        }
        return {static_cast<uint16_t>(q - first), how};
    }

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

    template <typename T>
    static constexpr bool run_frame = std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>;

    /// The NSS pad read low, the window open (yes at once without a pad):
    /// one load of the port's input register.
    [[gnu::always_inline]] static bool pad_low() {
        if constexpr (has_nss_pad) {
            return (NssPin::P::in() & NssPin::mask) == 0u;
        } else {
            return true;
        }
    }

    /// SPE up, inline: a load and a store at a run's opening.
    [[gnu::always_inline]] static void raise_spe() {
        S::regs().CTLR1 = static_cast<uint16_t>(S::regs().CTLR1 | spi_spe);
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
    /// select or the budget ran out, `how` saying which. FOUR POLLS A LOOK:
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

// The frame's time and the write-ahead threshold it decides: 16 cycles
// an 8-bit frame at /2, doubling per code; two frames in flight from
// /64 on 8-bit frames and from /32 on 16-bit ones, never at /32 and 8.
static_assert(spi_frame_cycles(SpiClock::div2, SpiDataSize::bits8) == 16u);
static_assert(spi_frame_cycles(SpiClock::div16, SpiDataSize::bits8) == 128u);
static_assert(spi_frame_cycles(SpiClock::div256, SpiDataSize::bits16) == 4096u);
static_assert(spi_write_ahead_from(SpiDataSize::bits8, spi_default_hold_off_cycles) ==
              SpiClock::div64);
static_assert(spi_write_ahead_from(SpiDataSize::bits16, spi_default_hold_off_cycles) ==
              SpiClock::div32);
static_assert(!spi_write_ahead_safe(SpiClock::div32, SpiDataSize::bits8, spi_default_hold_off_cycles));
static_assert(spi_write_ahead_safe(SpiClock::div256, SpiDataSize::bits8, spi_default_hold_off_cycles));
// A longer hold-off moves two-in-flight to a slower code: 600 cycles,
// a sum of 652, from /128 on 8-bit frames and from /64 on 16-bit ones.
static_assert(spi_write_ahead_from(SpiDataSize::bits8, 600u) == SpiClock::div128);
static_assert(spi_write_ahead_from(SpiDataSize::bits16, 600u) == SpiClock::div64);

// The client's modes: the CH32V006 refuses modes 1 and 2 (measured), the
// CH32V003 nothing yet (unmeasured).
static_assert(spi_client_mode_served(SpiMode::mode0) && spi_client_mode_served(SpiMode::mode3));
static_assert(spi_client_mode_served(SpiMode::mode1) == (device::part != Ch32Part::v006));

// Table 16-1.
static_assert(!spi_mode_cpol(SpiMode::mode1) && spi_mode_cpha(SpiMode::mode1));
static_assert(spi_mode_cpol(SpiMode::mode2) && !spi_mode_cpha(SpiMode::mode2));

// The refusals spi_config_valid() owes the chapter.
static_assert(spi_config_valid(SpiConfig{}));
static_assert(!spi_config_valid(SpiConfig{.direction = SpiDirection::half_duplex_out, .crc = true}));
static_assert(!spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0}));
static_assert(!spi_config_valid(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));
#if BRIO_CH32_PART_V006
static_assert(spi_pins_valid(spi1_default_pins) && spi_pins_valid(spi1_pins_for(4)));
static_assert(spi1_pins_for(2).sck == Pad{'D', 2} && spi1_pins_for(2).remap == 2u);
#else
static_assert(spi_pins_valid(spi1_default_pins) && spi_pins_valid(spi1_pins_for(1)));
static_assert(spi1_pins_for(1).nss == Pad{'C', 0} && spi1_pins_for(1).sck == Pad{'C', 5} && spi1_pins_for(1).remap == 1u);
#endif
static_assert(!spi_pins_valid(SpiPins{.sck = {'C', 5}, .mosi = {'C', 5}}));
static_assert(!spi_pins_valid(SpiPins{.mosi = {'C', 6}}));
static_assert(!spi_pins_valid(SpiPins{.sck = {'C', 5}, .mosi = {'C', 6}, .remap = 7}));

// The control words: mode 3 host at /16 under software select.
static_assert(spi_ctlr1_of(SpiConfig{.mode = SpiMode::mode3, .clock = SpiClock::div16}) ==
              (spi_cpha | spi_cpol | spi_mstr | (3u << 3) | spi_ssm | spi_ssi));
static_assert(spi_ctlr2_of(SpiConfig{.nss = SpiNss::hardware_output, .dma_receive = true}) ==
              (spi_ssoe | spi_rxdmaen));

} // namespace brio
