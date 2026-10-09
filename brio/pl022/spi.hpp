/*
 * spi.hpp
 *
 * THE ARM PRIMECELL SYNCHRONOUS SERIAL PORT (PL022), written once for
 * every family that carries it: an IP STRATUM, a directory named for a
 * peripheral DESIGN rather than for a silicon, sitting where a core
 * stratum sits - above util/, below the families (docs/pl022/README.md,
 * and the rule in docs/design/overview.md: "A core stratum sits between
 * util/ and the families that share a core").
 *
 * The three layers every brio bus driver has are all here:
 *
 *  Pl022Ssp<Chip, n>     the RESOURCE, named for the IP: which instance,
 *                        where its registers are, its reset and its
 *                        interrupt line (both asked of the family), the
 *                        two control registers under the block's rule
 *                        (written with SSE clear), the prescaler pair,
 *                        the data and status verbs, the interrupt trio
 *                        and the ISR body that reads the raised-and-
 *                        enabled sources. It decides nothing.
 *  Pl022Host<Chip, n, pins, TxEngine, RxEngine>
 *                        the ENGINE util/spi_bus.hpp's SpiBus drives.
 *                        Its Request is the other strata's VERBATIM (a
 *                        chip select, a D/C line, a command phase, a data
 *                        phase with optional out and in, the per-request
 *                        mode / rate / frame size, the polled or
 *                        ISR-pumped completion), so an application
 *                        written over SpiBus on another target runs here
 *                        unchanged.
 *  Pl022Client<Chip, n, pins>
 *                        the other end of the wire, a thin surface with
 *                        an ISR body: a client is a protocol and which
 *                        one is the application's.
 *
 * THIS FILE KNOWS NO CHIP, AND NO CORE EITHER. It includes kernel/ and
 * util/ and nothing else: no vendor header, no family header, no core
 * stratum, and it never names a reset controller, a clock, a pad
 * function, an interrupt controller or a DMA request. Even the
 * microsecond busy-wait a chip select's setup time is spent on arrives
 * as a TYPE of the family's, whose verb this file calls without naming
 * it - a PL022 may sit on a core brio/cortexm/ knows nothing about.
 * WHAT A FAMILY OWES IT is the `Pl022Chip` concept below,
 * satisfied by one traits type the family's own header defines; that
 * header also keeps the PUBLIC NAMES apps use (`Pl022<n>`, `SpiHost<n,
 * pins, ...>`, `SpiClient<n, pins>`) as aliases of these three
 * templates, so nothing above ever spells a Chip.
 *
 * WHAT THE IP DOES, and shapes the engine:
 *  - a host or a client, Motorola SPI, TI or Microwire framing, frames
 *    of 4 to 16 bits (DSS), two FIFOs whose DEPTH is a synthesis
 *    parameter and therefore the family's (`Chip::fifo_depth`);
 *  - the bit rate is SSPCLK / (CPSDVSR x (1 + SCR)), the prescaler EVEN
 *    in 2..254 and the serial clock rate 0..255, so the reachable
 *    divisors run from 2 to 65024 and a rate is REFUSED rather than
 *    rounded up past the slowest;
 *  - a CLIENT needs SSPCLK at least twelve times its own clock: the
 *    input is double-synchronized, which is the block's ratio and not a
 *    chip's;
 *  - CONTROL REGISTERS ARE WRITTEN WITH THE PORT DISABLED (SSE clear),
 *    so a change of mode, rate or width costs a disable/enable pair -
 *    paid only when something moved. LBM and SOD are the two live bits,
 *    changeable under SSE;
 *  - four maskable sources on ONE interrupt line per instance: the
 *    receive FIFO at or above half (RXIM), the RECEIVE TIMEOUT (RTIM: a
 *    frame waiting and no clock for 32 bit periods), the transmit FIFO
 *    at or below half (TXIM), the receive overrun (RORIM). Only the
 *    timeout and the overrun clear through SSPICR; the two FIFO levels
 *    clear by moving data;
 *  - one DMA request per FIFO (SSPDMACR);
 *  - LBM loops the transmitter into the receiver with nothing on the
 *    wire - a self-test with no peer, which is why it is a HOST verb
 *    here: the host re-applies its whole configuration at a request that
 *    changes mode, rate or width, so the loop-back has to be part of the
 *    applied state or it would be lost at the first such request.
 *
 * THE FIFOS ARE THE WRITE-AHEAD, on both completion styles. A FIFO's
 * worth of frames is in flight at once - written with no flag read, since
 * a transmit FIFO that holds fewer than its depth cannot be full and a
 * receive FIFO that is handed back no more than its depth cannot overrun
 * (12.3.3.4 and 12.3.3.5 of the RP2350 data sheet, the depth the
 * family's `fifo_depth`) - and every frame that comes BACK is the one
 * witness the bus has room for one more: the receive side counts what
 * the transmit side only promised.
 *
 * THE POLLED LOOP has two shapes. The RECEIVE shape (`run_duplex`)
 * primes the transmit FIFO with the first `fifo_depth` frames and then,
 * per frame, waits for RNE, reads the frame - stored, or thrown away
 * when the request has no in buffer - and writes the one `fifo_depth`
 * ahead: three accesses of the block a frame (SSPSR, SSPDR in, SSPDR
 * out), the frames in flight - written and not yet read - never more
 * than the receive FIFO holds, so no handler that holds the loop off can
 * overrun it. The TRANSMIT-ONLY shape (`run_write`) paces on TNF and
 * reads nothing per frame - two accesses a frame - letting the receive
 * FIFO overrun by design with RORIM masked, and at its tail waits for
 * BSY to clear, drains the receive FIFO and clears the overrun; that
 * tail is up to a FIFO's worth of reads after the wire, so the shape is
 * taken only for a write longer than the FIFO at the one rate where the
 * receive shape falls behind the wire (clk_peri / 2 on 8-bit frames,
 * measured on both of the RP2350's cores). No accessor call on either,
 * the width a template parameter, the buffers advancing pointers, the
 * waits bounded by ONE budget for the whole transaction. Both read every field through the Request they
 * were lent and copy nothing: a polled request completes inside
 * start(), so nothing of it is needed after the return
 * (util/bus_master.hpp's contract).
 *
 * THE PUMP (`fill` / `take`, the interrupt body) is the same window on
 * the interrupt: start() fills the FIFO and THEN arms the receive level
 * and the timeout - filling with the line already armed let the handler
 * run inside the fill and both sides count the same frames, which was
 * measured to overrun the receive FIFO at 37.5 MHz on the Hazard3 half.
 * Each receive interrupt (the FIFO at half, or the timeout for a tail)
 * takes what came back and writes as many more. THE TRANSMIT INTERRUPT
 * IS NEVER THE PUMP'S EDGE. What the pump needs after start() returned -
 * the three buffers, the counts, the two pins, the width - is copied
 * into `tenure_` as inline stores, never a call; the reply capsule is
 * the arbiter's and is not copied. The phase boundary drains: the first
 * data frame goes out only when the last command frame has come back, so
 * D/C flips between them on the wire. The receive FIFO is NOT flushed at
 * start(): every transaction takes back every frame it wrote, so a frame
 * can stand in it only after a fault, and the fault paths drain it.
 *
 * THE REQUEST IS LAID OUT WITHOUT PADDING: the four words first (the two
 * out buffers, the in buffer, the reply), then the two half-words (the
 * length, the prescaler pair), then the bytes - 28 bytes where the
 * family's PinRef is one byte, and every copy the kernel and the arbiter
 * make of it is that much shorter. The field NAMES are the other
 * strata's and do not move.
 *
 * `apply()` IS ONE COMPARE: the prescaler pair, the mode and the width
 * are folded into the word the block takes (SSPCPSR over SSPCR0,
 * `spi_control_word`) and held against the word last applied; only a
 * change pays the disable/enable pair the chapter demands. A request
 * above the bus's ceiling is clamped on the slow path alone.
 *
 * THE SELECTS. The host's chip select is a pin the Request carries
 * (docs/design/spi-bus.md's arrangement on every target), never the
 * block's own SSPFSSOUT, which in Motorola format PULSES BETWEEN FRAMES
 * - a device reading a multi-frame transaction inside one select window
 * cannot take that. The CLIENT's select IS the pad: the PL022 in slave
 * mode frames on SSPFSSIN, so a client's `cs` goes to the peripheral and
 * `selected()` reads the same pad. And with SPH = 0 (modes 0 and 2) the
 * client frames on the SELECT: it takes ONE frame per select window and
 * ignores the rest while the select stays low, so a host holding a
 * select for a whole transaction reaches it for more than one frame only
 * in modes 1 and 3.
 *
 * THE DARK LISTENER IS THE PAD AND NOT SOD. `Pl022Client::drive_output`
 * hands the transmit pad to the peripheral or takes it back as an
 * undriven input, because what SOD does to the PAD is a fact of the chip
 * around the block and not of the block: the block's pad-enable output
 * nSSPOE reaches no pad on either silicon this file has met, so both
 * leave the pad driven under SOD, and a chip that wired it would release
 * it. Releasing the pad is right either way, so that is what this file does;
 * `sod()` stays a bit of the resource, for a program that wants to see
 * what its own silicon does with it.
 *
 * THE ENGINE SLOTS: a transmit and a receive engine, both or neither
 * (the data phase is full duplex and the transaction's completion is the
 * RECEIVE block's), told the instance's two requests at arm(). The data
 * phase of a request moves as one block each way, the command phase on
 * the pump. ONE INTERRUPT A TRANSACTION: every frame that came back was
 * sent, so the receive block's completion proves the transmit block's,
 * and the transmit engine is armed to report a bus error and nothing
 * else (the family's DmaReport::errors). THE BEAT IS THE FRAME: engines
 * whose element is 16 bits wide - SSPDR's width - carry both frame sizes,
 * each block at its frame's width; byte engines carry 8-bit frames, and a
 * 16-bit request then falls back to the pump, as does one whose buffers
 * are not aligned to a half-word. FRAMES IN A BYTE BUFFER: one byte per
 * 8-bit frame, two bytes low-first per 16-bit frame - the other strata's
 * rule, which on a little-endian core IS a half-word in memory.
 *
 * NOT built, each with its reason in docs/pl022/README.md: the block's
 * own frame signal as a host select, the National Semiconductor
 * Microwire transaction (half duplex, a control frame and then the
 * slave's answer, where this engine's Request is full duplex and one
 * frame wide), the client on the DMA engines.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <concepts>
#include <optional>
#include <type_traits>
#include <utility>

#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// ---- what a family owes this file ---------------------------------------------

/**
 * The absent engine, as the concept measures a family's
 * `engines_distinct` against it: `present` is all this file ever asks of
 * an engine slot, and a family's own empty-slot tag says the same.
 */
struct Pl022AbsentEngine {
    Pl022AbsentEngine() = delete;
    static constexpr bool present = false;
};

/**
 * THE CONTRACT: what a family's chip-traits type states so that this
 * file can drive the block without knowing which silicon carries it.
 * Every member is here because some line below needs it.
 *
 * Two things this concept names and cannot check here: `sspclk_hz(clock)`,
 * a template over the family's OWN clock types, of which this file knows
 * none; and the VERB over `DelayRate`, which is found by argument-
 * dependent lookup on a type this file only has as a name. Each has a
 * concept of its own - `Pl022ChipClock` and `Pl022ChipDelay` - checked
 * where the thing is in hand, which for both is the host.
 */
template <typename C>
concept Pl022Chip =
    requires {
        /// The register block of one instance, as the family's device
        /// description spells it: the member names are the PL022's own
        /// (SSPCR0, SSPCR1, SSPDR, SSPSR, ...).
        typename C::Regs;
        /// What the family's interrupt controller calls a line.
        typename C::Irq;
        /// That controller: enable / disable one line. A family may carry
        /// an NVIC under one architecture and something else under
        /// another, so this file never names one.
        typename C::Interrupts;
        /// The family's pin-set type, one pad per signal: which pads
        /// exist and which function routes an SPI to them is the
        /// family's table, never this file's.
        typename C::Pins;
        /// A pin named at RUN TIME - what a Request carries as its chip
        /// select and its D/C line, since those are the application's
        /// pins and not the peripheral's.
        typename C::PinRef;
        /// What the family's DMA calls a peripheral request.
        typename C::DmaRequest;
        /// The family's microsecond busy-wait, as the precomputed factor
        /// a caller stores once: `cs_setup_us` is spent on it. The VERB
        /// over this type is `Pl022ChipDelay` below.
        typename C::DelayRate;
        /// How many instances this family carries, how deep the two
        /// FIFOs were synthesized, and what the pin set puts in a signal
        /// that is not routed.
        { C::instances } -> std::convertible_to<uint8_t>;
        { C::fifo_depth } -> std::convertible_to<uint8_t>;
        { C::no_pad } -> std::convertible_to<uint8_t>;
        /// Below how many data frames the pump beats the engines: the
        /// engines' fixed cost per transaction over the pump's cost per
        /// frame, both MEASURED on the family's own core (its document
        /// states the two inputs). A data phase shorter than this takes
        /// the pump even on a host with engines bound.
        { C::dma_min_frames } -> std::convertible_to<uint16_t>;
    } &&
    requires(volatile uint32_t& reg, uint32_t bits, typename C::Pins pins, uint8_t instance,
             typename C::Irq line, typename C::PinRef line_out, uint32_t hz) {
        /// Instance 0 answers every per-instance question; so does every
        /// other instance the family has. Each is a template on the
        /// instance number so the answer is a compile-time constant -
        /// a register address, a line, a request.
        { C::template regs<0>() } -> std::same_as<typename C::Regs&>;
        { C::template irq<0>() } -> std::same_as<typename C::Irq>;
        /// The block from its reset state, held, and whether it is out:
        /// on one family a reset controller, on another a clock gate.
        { C::template reset<0>() } -> std::same_as<bool>;
        { C::template released<0>() } -> std::same_as<bool>;
        C::template hold<0>();
        /// The two request numbers a host hands its engines.
        { C::template tx_request<0>() } -> std::same_as<typename C::DmaRequest>;
        { C::template rx_request<0>() } -> std::same_as<typename C::DmaRequest>;
        /// One bus write that sets or clears bits of a register, with no
        /// read-modify-write: the enable and the two live bits share
        /// SSPCR1, and the interrupt mask is touched from two contexts.
        C::set_bits(reg, bits);
        C::clear_bits(reg, bits);
        /// The pads: is this pin set legal for this instance, and which
        /// pad carries each signal (`no_pad` for one the set leaves out).
        { C::pins_valid(instance, pins) } -> std::same_as<bool>;
        { C::sck_pad(pins) } -> std::same_as<uint8_t>;
        { C::tx_pad(pins) } -> std::same_as<uint8_t>;
        { C::rx_pad(pins) } -> std::same_as<uint8_t>;
        { C::cs_pad(pins) } -> std::same_as<uint8_t>;
        /// Then the PAD ITSELF as a type, with the function code that
        /// routes an SPI to it and the electrical setup each signal
        /// wants; the three verbs this file calls on a pad are handing it
        /// over, taking it back and reading its level (a client's select).
        /// The two receive pads are asked SEPARATELY because they are two
        /// electrical situations: a host's answer line may have nothing
        /// driving it (no client, or one listening dark), while a
        /// client's command line is driven by the host whenever it
        /// matters.
        C::template Pad<0>::function(C::pad_function, C::sck_pad_config());
        C::template Pad<0>::function(C::pad_function, C::tx_pad_config());
        C::template Pad<0>::function(C::pad_function, C::host_rx_pad_config());
        C::template Pad<0>::function(C::pad_function, C::client_rx_pad_config());
        C::template Pad<0>::function(C::pad_function, C::cs_pad_config());
        { C::template Pad<0>::read() } -> std::same_as<bool>;
        C::template Pad<0>::release();
        /// The run-time pin, driven high and low around a transaction.
        line_out.set();
        line_out.clear();
        /// Two engines of one host must not name one DMA channel; what
        /// "the same channel" means is the family's DMA's business.
        { C::template engines_distinct<Pl022AbsentEngine, Pl022AbsentEngine>() }
            -> std::same_as<bool>;
        /// The busy-wait's factor for a rate, computed once per clock
        /// change so no division runs at wait time.
        { C::delay_rate(hz) } -> std::same_as<typename C::DelayRate>;
        /// The line's two verbs, on the family's own controller.
        C::Interrupts::enable(line);
        C::Interrupts::disable(line);
    };

/**
 * WHICH CLOCK OF THIS FAMILY'S TREE IS SSPCLK - the half of the contract
 * that is a template over the family's clock types and so can only be
 * checked where a clock is in hand (Pl022Host::init).
 */
template <typename C, typename Clock>
concept Pl022ChipClock = requires(Clock clock) {
    { C::sspclk_hz(clock) } -> std::same_as<uint32_t>;
};

/**
 * THE VERB THAT BELONGS TO `DelayRate`, and so cannot be named here: a
 * microsecond busy-wait is made of a core's own counter, and the PL022
 * sits on cores this file must know nothing about. The family hands over
 * its own type and this file calls that type's verb, found where the
 * type is - which is why `delay_us` is written unqualified and why the
 * check is a concept of its own, where the type is in hand.
 */
template <typename C>
concept Pl022ChipDelay = requires(typename C::DelayRate rate, uint32_t us) {
    { delay_us(rate, us) } -> std::convertible_to<bool>;
};

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

/// The bit rate: sspclk / (cpsdvsr x (1 + scr)), cpsdvsr even in
/// 2..254, scr 0..255.
struct SpiClock {
    uint8_t cpsdvsr = 2;
    uint8_t scr = 7;
    constexpr bool valid() const { return cpsdvsr >= 2u && (cpsdvsr & 1u) == 0u; }
    constexpr uint32_t divisor() const { return static_cast<uint32_t>(cpsdvsr) * (static_cast<uint32_t>(scr) + 1u); }
    constexpr bool operator==(const SpiClock& o) const { return cpsdvsr == o.cpsdvsr && scr == o.scr; }
};

/// The named divisors the other strata's requests speak (the reference
/// rate over a power of two), and any even divisor up to 65024.
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

// ---- the register vocabulary --------------------------------------------------------

/// SSPCR0: the frame, the framing, the phase pair and the serial clock
/// rate.
struct SpiControl0 {
    static constexpr uint32_t data_size_lsb = 0;      ///< DSS: the frame's bits, less one
    static constexpr uint32_t data_size_bits = 0xFu << data_size_lsb;
    static constexpr uint32_t format_lsb = 4;         ///< FRF
    static constexpr uint32_t cpol = 1u << 6;         ///< SPO
    static constexpr uint32_t cpha = 1u << 7;         ///< SPH
    static constexpr uint32_t rate_lsb = 8;           ///< SCR
    static constexpr uint32_t rate_bits = 0xFFu << rate_lsb;
};

/// SSPCR1: the enable, the role, and the two bits that are LIVE under it.
struct SpiControl1 {
    static constexpr uint32_t loopback = 1u << 0;        ///< LBM
    static constexpr uint32_t enable = 1u << 1;          ///< SSE
    static constexpr uint32_t client = 1u << 2;          ///< MS
    static constexpr uint32_t output_disable = 1u << 3;  ///< SOD
};

/// SSPCPSR: the even prescaler, and SSPDR's frame field.
struct SpiDataField {
    static constexpr uint32_t prescaler = 0xFFu;
    static constexpr uint32_t frame = 0xFFFFu;
};

/// SSPSR's flags.
struct SpiFlag {
    static constexpr uint32_t tx_empty = 1u << 0;
    static constexpr uint32_t tx_not_full = 1u << 1;
    static constexpr uint32_t rx_not_empty = 1u << 2;
    static constexpr uint32_t rx_full = 1u << 3;
    static constexpr uint32_t busy = 1u << 4;
};

/// The four interrupt sources: one layout for IMSC, RIS, MIS and ICR
/// (only the timeout and the overrun clear through ICR; the two FIFO
/// levels clear by moving data).
struct SpiInterrupt {
    static constexpr uint32_t overrun = 1u << 0;     ///< RORIM: the receive FIFO overflowed
    static constexpr uint32_t rx_timeout = 1u << 1;  ///< RTIM: data waiting, 32 bit periods idle
    static constexpr uint32_t rx = 1u << 2;          ///< RXIM: the receive FIFO at or above half
    static constexpr uint32_t tx = 1u << 3;          ///< TXIM: the transmit FIFO at or below half
    static constexpr uint32_t all = tx | rx | rx_timeout | overrun;
};

/// SSPDMACR: the two request enables.
struct SpiDmaControl {
    static constexpr uint32_t rx = 1u << 0;
    static constexpr uint32_t tx = 1u << 1;
};

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
    uint32_t v = static_cast<uint32_t>(c.clock.scr) << SpiControl0::rate_lsb;
    v |= static_cast<uint32_t>(c.format) << SpiControl0::format_lsb;
    v |= static_cast<uint32_t>(c.bits - 1u) << SpiControl0::data_size_lsb;
    if (c.format == SpiFormat::motorola) {
        if (spi_mode_cpol(c.mode)) { v |= SpiControl0::cpol; }
        if (spi_mode_cpha(c.mode)) { v |= SpiControl0::cpha; }
    }
    return v;
}

/// SSPCR1 for a configuration, SSE clear.
constexpr uint32_t spi_cr1_of(const SpiConfig& c) {
    uint32_t v = 0;
    if (c.role == SpiRole::client) { v |= SpiControl1::client; }
    if (c.loopback) { v |= SpiControl1::loopback; }
    if (c.output_disabled) { v |= SpiControl1::output_disable; }
    return v;
}

// =============================================================================
// The resource
// =============================================================================

template <Pl022Chip Chip, uint8_t n>
struct Pl022Ssp {
    static_assert(n < Chip::instances, "this family has no PL022 with that number");
    Pl022Ssp() = delete;

    using Regs = typename Chip::Regs;

    static constexpr uint8_t index = n;
    static constexpr uint8_t fifo_depth = Chip::fifo_depth;
    static constexpr typename Chip::DmaRequest dreq_tx = Chip::template tx_request<n>();
    static constexpr typename Chip::DmaRequest dreq_rx = Chip::template rx_request<n>();

    static Regs& regs() { return Chip::template regs<n>(); }
    static constexpr typename Chip::Irq irq() { return Chip::template irq<n>(); }
    static volatile void* data_address() { return &regs().SSPDR; }

    /// The block from its reset state: held, released, ready.
    static bool reset() { return Chip::template reset<n>(); }
    static bool released() { return Chip::template released<n>(); }
    static void hold() { Chip::template hold<n>(); }

    static bool enabled() { return (regs().SSPCR1 & SpiControl1::enable) != 0u; }
    static void enable(bool on) {
        if (on) { Chip::set_bits(regs().SSPCR1, SpiControl1::enable); }
        else { Chip::clear_bits(regs().SSPCR1, SpiControl1::enable); }
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

    /// LBM and SOD are the two live bits of SSPCR1: allowed under SSE.
    static void loopback(bool on) {
        if (on) { Chip::set_bits(regs().SSPCR1, SpiControl1::loopback); }
        else { Chip::clear_bits(regs().SSPCR1, SpiControl1::loopback); }
    }
    static bool loopback() { return (regs().SSPCR1 & SpiControl1::loopback) != 0u; }
    static void output_disabled(bool on) {
        if (on) { Chip::set_bits(regs().SSPCR1, SpiControl1::output_disable); }
        else { Chip::clear_bits(regs().SSPCR1, SpiControl1::output_disable); }
    }

    /// The rate as the registers hold it.
    static SpiClock clock() {
        return SpiClock{static_cast<uint8_t>(regs().SSPCPSR & SpiDataField::prescaler),
                        static_cast<uint8_t>((regs().SSPCR0 & SpiControl0::rate_bits) >> SpiControl0::rate_lsb)};
    }
    static uint8_t bits() { return static_cast<uint8_t>((regs().SSPCR0 & SpiControl0::data_size_bits) + 1u); }

    // ---- data and status ------------------------------------------------------
    static uint32_t flags() { return regs().SSPSR; }
    static bool tx_not_full() { return (flags() & SpiFlag::tx_not_full) != 0u; }
    static bool tx_empty() { return (flags() & SpiFlag::tx_empty) != 0u; }
    static bool rx_not_empty() { return (flags() & SpiFlag::rx_not_empty) != 0u; }
    static bool rx_full() { return (flags() & SpiFlag::rx_full) != 0u; }
    static bool busy() { return (flags() & SpiFlag::busy) != 0u; }

    static void write_data(uint16_t v) { regs().SSPDR = v; }
    static uint16_t read_data() { return static_cast<uint16_t>(regs().SSPDR & SpiDataField::frame); }
    /// Throw away whatever the receive FIFO holds.
    static void flush_rx() {
        while (rx_not_empty()) {
            (void)regs().SSPDR;
        }
    }

    // ---- interrupts -----------------------------------------------------------
    static void interrupts(uint32_t mask, bool on) {
        if (on) { Chip::set_bits(regs().SSPIMSC, mask); } else { Chip::clear_bits(regs().SSPIMSC, mask); }
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
        regs().SSPDMACR = (tx ? SpiDmaControl::tx : 0u) | (rx ? SpiDmaControl::rx : 0u);
    }
};

// =============================================================================
// The host engine
// =============================================================================

/// A DMA block the engines could not finish, or a polled block that
/// never completed: the engine's own status code, in the range
/// util/bus_master.hpp leaves to engines.
inline constexpr uint8_t spi_dma_fault = bus_engine_status;

/// SSPCPSR over SSPCR0 as ONE word - the prescaler in bits 23:16, the
/// control register's own sixteen below - for a Motorola host at one
/// prescaler pair, mode and width: the word `apply()` compares, computed
/// from a request's three fields in a handful of shifts. The polarity
/// is the mode's bit 1 into SPO (bit 6), the phase its bit 0 into SPH
/// (bit 7), DSS is 7 for a byte and 15 for a half-word; FRF stays zero.
[[gnu::always_inline]] inline constexpr uint32_t spi_control_word(SpiClock c, SpiMode m, SpiDataSize bits) {
    return (static_cast<uint32_t>(c.cpsdvsr) << 16) |
           (static_cast<uint32_t>(c.scr) << SpiControl0::rate_lsb) |
           ((static_cast<uint32_t>(m) & 2u) << 5) | ((static_cast<uint32_t>(m) & 1u) << 7) |
           (7u | (static_cast<uint32_t>(bits) << 3));
}
// The folded word is what configure() writes, held against the long way.
static_assert(spi_control_word(SpiClocks::div16, SpiMode::mode3, SpiDataSize::bits16) ==
              ((2u << 16) | spi_cr0_of({.mode = SpiMode::mode3, .bits = 16, .clock = SpiClocks::div16})));
static_assert(spi_control_word(SpiClock{254, 255}, SpiMode::mode2, SpiDataSize::bits8) ==
              ((254u << 16) | spi_cr0_of({.mode = SpiMode::mode2, .bits = 8, .clock = SpiClock{254, 255}})));
static_assert(spi_control_word(SpiClocks::div2, SpiMode::mode1, SpiDataSize::bits8) ==
              ((2u << 16) | spi_cr0_of({.mode = SpiMode::mode1, .bits = 8, .clock = SpiClocks::div2})));

/**
 * Pl022Host<Chip, n, pins, TxEngine, RxEngine>
 *
 * The engine SpiBus (util/spi_bus.hpp = BusMaster) drives; the family's
 * own header aliases it to `SpiHost<n, pins, ...>` and carries the
 * example in that family's spelling. Its Request is the other strata's
 * (the file header): the bus AO asserts the request's own chip select
 * around the transaction, a COMMAND phase goes out with D/C low and a
 * DATA phase with it high, the data phase has an optional out buffer
 * (null = 0xFF dummies) and an optional in buffer (null = the frames
 * read back are discarded), and every buffer is LENT until the reply
 * lands. The mode, rate and frame size travel per request; the bit order
 * is the PL022's (most significant first, no other) and no verb offers
 * another.
 */
template <Pl022Chip Chip, uint8_t n, typename Chip::Pins pins, typename TxEngine, typename RxEngine>
class Pl022Host {
    using S = Pl022Ssp<Chip, n>;
    using Regs = typename Chip::Regs;
    static_assert(Chip::pins_valid(n, pins),
                  "brio SpiHost: these pads cannot carry this instance's signals - which pad "
                  "carries which signal of which SPI is fixed per instance, each under the one "
                  "function that routes the block to it, so see the family's own pin table");
    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0, "the engine slots must name a complete type");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is full-duplex and "
                  "its completion is the RECEIVE block's");
    static_assert(Chip::template engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the two engines must ride two different DMA channels");
    static_assert(Pl022ChipDelay<Chip>,
                  "the family's chip traits must hand over a DelayRate its own delay_us takes: "
                  "a Request's cs_setup_us is spent on it (Pl022ChipDelay)");
    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, typename RxEngine::element> &&
                   (std::is_same_v<typename TxEngine::element, uint8_t> ||
                    std::is_same_v<typename TxEngine::element, uint16_t>);
        } else {
            return true;
        }
    }(), "brio SpiHost: both DMA engines carry the same element, uint8_t or uint16_t - the widest "
         "beat SSPDR takes is sixteen bits; uint16_t engines carry 16-bit frames too, uint8_t "
         "ones leave them to the pump");

    static constexpr uint8_t sck_pin = Chip::sck_pad(pins);
    static constexpr uint8_t tx_pin = Chip::tx_pad(pins);
    static constexpr uint8_t rx_pin = Chip::rx_pad(pins);
    static_assert(tx_pin != Chip::no_pad, "brio SpiHost: a host needs SCK and TX; RX is optional for a write-only bus");

    using SckPad = typename Chip::template Pad<sck_pin>;
    using TxPad = typename Chip::template Pad<tx_pin>;
    using RxPad = typename Chip::template Pad<rx_pin != Chip::no_pad ? rx_pin : sck_pin>;

public:
    Pl022Host() = delete;
    using Resource = S;
    static constexpr typename Chip::Pins pin_set = pins;
    static constexpr bool has_engines = TxEngine::present;
    /// Engines whose element is a half-word: 16-bit frames ride them.
    static constexpr bool wide_engines = [] {
        if constexpr (TxEngine::present) {
            return sizeof(typename TxEngine::element) == 2u;
        } else {
            return false;
        }
    }();
    /// Below how many data frames a request takes the pump even with
    /// engines bound: the family's measured pair (the concept).
    static constexpr uint16_t dma_min_frames = Chip::dma_min_frames;

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

    /// The transaction descriptor - the other strata's fields under the
    /// other strata's names, laid out words first so that nothing is
    /// padded but the tail (the file header).
    struct Request {
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        ReplyTo<SpiDone> reply;
        uint16_t len;      ///< phase 2 length, in FRAMES
        /// Per-transaction bus configuration: a shared bus's devices
        /// each name their own, and the engine reprograms the peripheral
        /// only when something CHANGED.
        SpiClock clock = SpiClocks::div16;
        typename Chip::PinRef cs;   ///< asserted low around the transaction
        typename Chip::PinRef dc;   ///< display D/C line; null = no such pin
        /// Microseconds between the CS assertion and the first clock -
        /// what a device's datasheet calls CS setup. Spent spinning in
        /// start(), main context; 0 = none.
        uint8_t cs_setup_us = 0;
        uint8_t cmd_len;   ///< in FRAMES
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
    /// - the divisor is of whichever rate the family's traits call
    /// SSPCLK. `max_sck_hz` is an optional CEILING for the whole bus;
    /// 0 = none. False when the ceiling cannot be produced or the block
    /// did not come up.
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(Pl022ChipClock<Chip, Clock>,
                      "the family's chip traits must say which rate of this Clock type is "
                      "SSPCLK (Pl022ChipClock's sspclk_hz)");
        static_assert(clock_follows<Clock, Pl022Host>(),
                      "this SpiHost is initialized with a DynamicClock that does not list it "
                      "among its Users: its SCK ceiling and its cs_setup timing would go stale");
        Chip::Interrupts::disable(S::irq());
        ceiling_hz_ = max_sck_hz;
        rebase(Chip::sspclk_hz(clock), clock_hz(clock));
        if (max_sck_hz != 0u && !ceiling_) {
            return false;
        }
        if (!S::reset()) {
            return false;
        }
        applied_ = boot_config();
        applied_word_ = spi_control_word(applied_.clock, applied_.mode, SpiDataSize::bits8);
        if (!S::configure(applied_)) {
            return false;
        }
        status_ = spi_ok;
        S::interrupts(SpiInterrupt::all, false);
        if constexpr (has_engines) {
            // The receive block's completion is the transaction's, and the
            // proof of the transmit block's: that one reports errors alone.
            // The transmit engine at the family's NORMAL level: a starved
            // transmit holds SCK between two frames and loses nothing.
            TxEngine::arm(S::data_address(), S::dreq_tx, false, TxEngine::Report::errors);
            arm_rx_engine();
        }
        // The pads go to the peripheral with the registers already
        // holding the idle polarity, then SSE: a pad handed over first
        // would show whatever an unconfigured block drives.
        SckPad::function(Chip::pad_function, Chip::sck_pad_config());
        TxPad::function(Chip::pad_function, Chip::tx_pad_config());
        if constexpr (rx_pin != Chip::no_pad) {
            RxPad::function(Chip::pad_function, Chip::host_rx_pad_config());
        }
        S::enable(true);
        S::flush_rx();
        Chip::Interrupts::enable(S::irq());
        return true;
    }

    /// The reference rate and the core rate changed. A Request's `clock`
    /// is a DIVISION of SSPCLK and scales by itself; what is recomputed
    /// is the ceiling and the cs_setup timing. THE BUS MUST BE IDLE.
    static void rebase(uint32_t pclk_hz, uint32_t sys_hz) {
        pclk_hz_ = pclk_hz;
        ceiling_ = ceiling_hz_ != 0u ? spi_clock_for(pclk_hz, ceiling_hz_) : std::optional<SpiClock>{};
        cs_rate_ = Chip::delay_rate(sys_hz);
    }
    /// The setting that produces at most `hz` of SCK at the SSPCLK last
    /// seen - the chooser a device's datasheet limit is spoken to.
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
        reconfigure(clamp(c), m, bits);
    }

    // ---- the transfer -------------------------------------------------------

    /// Begin a transaction (SpiBus calls it from main context). True when
    /// it completed SYNCHRONOUSLY (polled requests, the empty one); false
    /// when it runs on the ISR and a TransferDone follows - exactly
    /// util/bus_master.hpp's engine contract. The request is LENT for the
    /// call: the polled path reads it and keeps nothing, the asynchronous
    /// one copies what its tenure needs (the file header).
    static bool start(const Request& r) {
        status_ = spi_ok;
        const uint16_t total = static_cast<uint16_t>(static_cast<uint16_t>(r.cmd_len) + r.len);
        if (total == 0u) {
            return true;
        }
        apply(r);
        const bool has_cmd = r.cmd_len != 0u;
        if (has_cmd) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();   // assert, active low
        if (r.cs_setup_us != 0u) {
            (void)delay_us(cs_rate_, r.cs_setup_us);
        }
        if (r.polled) {
            // THE POLLED PATH. This instance's line is quiet already: the
            // pump arms it for its own tenure and disarms it at the end,
            // and the arbiter never overlaps two tenures.
            uint32_t budget = spin_budget(total);
            bool ok = true;
            if (has_cmd) {
                ok = run_polled(r.cmd.get(), nullptr, r.cmd_len, r.bits, budget);
                r.dc.set();
            }
            if (ok && r.len != 0u) {
                if constexpr (has_engines) {
                    if (engines_serve(r)) {
                        take_tenure(r, total);
                        launch_dma();
                        spin_dma();
                        r.cs.set();
                        return true;
                    }
                }
                ok = run_polled(r.tx.get(), r.rx.get(), r.len, r.bits, budget);
            }
            if (!ok) {
                settle_fault();
            }
            r.cs.set();
            return true;
        }
        // THE ASYNCHRONOUS PATH: the tenure's fields copied, the FIFO
        // filled, and only THEN the line armed (the file header).
        take_tenure(r, total);
        if constexpr (has_engines) {
            if (engines_serve(r)) {
                tenure_.engines = true;
                if (!has_cmd) {
                    launch_dma();
                    return false;   // dma_isr() is the completion edge
                }
            }
        }
        fill();
        S::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, true);
        return false;
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
        Tenure& t = tenure_;
        if constexpr (has_engines) {
            if (t.engines && !t.in_cmd && !dma_active_) {
                // The command phase is done on the pump: the engines take
                // the data phase.
                S::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, false);
                launch_dma();
                return false;
            }
        }
        if (t.received >= t.total) {
            S::interrupts(SpiInterrupt::rx | SpiInterrupt::rx_timeout, false);
            t.req.cs.set();
            return true;
        }
        fill();
        return false;
    }

    /// The DMA line's interrupt body - call it from the line the engines
    /// report on. A bus error on either channel ends the transaction
    /// with spi_dma_fault. Compiles away on an engineless host. True
    /// when the transaction just completed. The transmit channel reports
    /// a bus error and nothing else (init()), so a block of its own on
    /// the line IS an error.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            if ((TxEngine::service() & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                return finish_dma(spi_dma_fault);
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
        tenure_.req.cs.set();
        if constexpr (has_engines) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            arm_rx_engine();
        }
        Chip::Interrupts::disable(S::irq());
        tenure_.in_cmd = false;
        tenure_.engines = false;
        dma_active_ = false;
        dma_done_ = false;
        S::enable(false);
        const bool ok = S::reset();
        const bool cfg = S::configure(applied_);
        S::interrupts(SpiInterrupt::all, false);
        S::enable(true);
        S::flush_rx();
        Chip::Interrupts::enable(S::irq());
        return ok && cfg;
    }

    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Chip::Interrupts::disable(S::irq());
        S::interrupts(SpiInterrupt::all, false);
        S::enable(false);
        SckPad::release();
        TxPad::release();
        if constexpr (rx_pin != Chip::no_pad) {
            RxPad::release();
        }
        S::hold();
    }

private:
    /// What the asynchronous path keeps of a request after start()
    /// returned - the pump's and the engines' working set: the lent
    /// request copied AS WORDS (one load and one store a word, inline on
    /// every core - a struct assignment of this size is a memcpy call on
    /// a RISC-V build at -Os) and the two advancing pointers and counts
    /// the pump keeps beside it.
    struct Tenure {
        Request req;
        const uint8_t* out;        ///< the current phase's out buffer, advancing; null = 0xFFFF
        uint8_t* in;               ///< the data phase's in buffer, advancing; null = discarded
        uint16_t total;            ///< cmd_len + len, in frames
        uint16_t phase_end;        ///< cmd_len while the command phase runs, then total
        uint16_t sent;
        uint16_t received;
        bool in_cmd;               ///< the command phase has not come back whole
        bool engines;              ///< the data phase rides the engines
    };
    static_assert(sizeof(Request) % 4u == 0u, "the Request is copied as words");
    static constexpr size_t request_words = sizeof(Request) / 4u;

    /// The request's words, one load and one store each: the alignment is
    /// the Request's own (it holds pointers), stated to the compiler so
    /// that a strict-alignment core moves words and not bytes, and the
    /// four-byte copies are what every compiler expands in place.
    template <size_t... I>
    [[gnu::always_inline]] static inline void copy_words(Request& to, const Request& from,
                                                         std::index_sequence<I...>) {
        uint8_t* d = static_cast<uint8_t*>(__builtin_assume_aligned(&to, 4));
        const uint8_t* s = static_cast<const uint8_t*>(__builtin_assume_aligned(&from, 4));
        (__builtin_memcpy(d + 4u * I, s + 4u * I, 4u), ...);
    }

    static void take_tenure(const Request& r, uint16_t total) {
        Tenure& t = tenure_;
        copy_words(t.req, r, std::make_index_sequence<request_words>{});
        const bool has_cmd = r.cmd_len != 0u;
        t.out = has_cmd ? r.cmd.get() : r.tx.get();
        t.in = r.rx.get();
        t.total = total;
        t.phase_end = has_cmd ? r.cmd_len : total;
        t.sent = 0;
        t.received = 0;
        t.in_cmd = has_cmd;
        t.engines = false;
    }

    /// The engines serve a data phase no shorter than dma_min_frames, in
    /// 8-bit frames, and in 16-bit frames when they are half-word engines
    /// and both buffers are aligned to a half-word (a null one is): a
    /// misaligned run is one the DMA would move wrongly and refuses, so
    /// the pump takes it instead.
    static bool engines_serve(const Request& r) {
        if (r.len < dma_min_frames) {
            return false;
        }
        if (!spi_frame_is_halfword(r.bits)) {
            return true;
        }
        if constexpr (wide_engines) {
            const uintptr_t joint = reinterpret_cast<uintptr_t>(r.tx.get()) |
                                    reinterpret_cast<uintptr_t>(r.rx.get());
            return (joint & 1u) == 0u;
        } else {
            return false;
        }
    }

    static void launch_dma() {
        dma_done_ = false;
        dma_active_ = true;
        if constexpr (wide_engines) {
            if (spi_frame_is_halfword(tenure_.req.bits)) {
                launch_frames<uint16_t>();
                return;
            }
        }
        launch_frames<uint8_t>();
    }

    /// The data phase as one block each way, F the frame's beat. THE
    /// REQUESTS ARE RAISED AROUND THE ENGINES AND NOWHERE ELSE (the file
    /// header). The receive channel first: its request rises only when a
    /// frame has come back, and the transmit side is what starts the
    /// clock. The byte buffers are handed over as runs of F: a 16-bit
    /// frame low-first IS a half-word in memory, and engines_serve() has
    /// checked the alignment that makes the address one.
    template <typename F>
    static void launch_frames() {
        const Request& r = tenure_.req;
        if (r.rx.get() != nullptr) {
            (void)RxEngine::start(reinterpret_cast<F*>(r.rx.get()), r.len);
        } else {
            (void)RxEngine::start_discard(reinterpret_cast<F*>(&rx_sink_), r.len);
        }
        S::dma_requests(false, true);
        if (r.tx.get() != nullptr) {
            (void)TxEngine::start(reinterpret_cast<const F*>(r.tx.get()), r.len);
        } else {
            (void)TxEngine::start_fixed(reinterpret_cast<const F*>(&tx_dummy_), r.len);
        }
        S::dma_requests(true, true);
    }

    /// The receive engine bound, at the family's HIGH level - the rule's
    /// for an overrunning receive (docs/design/dma.md): the host's clock
    /// does not wait for a starved receive channel, and a frame that lands
    /// on a full receive FIFO is lost (SSPRIS.RORRIS). Every bind of it is
    /// this one, so the level is stated once.
    static void arm_rx_engine() {
        if constexpr (has_engines) {
            RxEngine::arm(S::data_address(), S::dreq_rx, true);
        }
    }

    /// One exit for the data phase. True when the ISR-style caller
    /// should post completion.
    static bool finish_dma(uint8_t st) {
        S::dma_requests(false, false);
        if (st != spi_ok) {
            status_ = st;
            (void)TxEngine::abandon();
            RxEngine::stop();
            arm_rx_engine();
        } else if (TxEngine::busy()) {
            // The receive block completing is proof the transmit one did.
            (void)TxEngine::complete();
        }
        dma_active_ = false;
        dma_done_ = true;
        if (!tenure_.req.polled) {
            tenure_.req.cs.set();
            return true;
        }
        return false;
    }

    /// The polled request's wait on the DMA completion - bounded (the
    /// slowest frame is 65024 x 16 reference cycles; the budget scales).
    static void spin_dma() {
        uint32_t spins = spin_budget(tenure_.req.len);
        while (!dma_done_ && spins-- != 0u) {
        }
        if (!dma_done_) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            arm_rx_engine();
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

    /// Put the peripheral where this request wants it: ONE compare of
    /// the folded word against the one applied (the file header). A
    /// request whose word differs goes the long way - clamped to the
    /// ceiling, compared again, written only if it still differs.
    [[gnu::always_inline]] static void apply(const Request& r) {
        if (spi_control_word(r.clock, r.mode, r.bits) == applied_word_) {
            return;
        }
        reconfigure(clamp(r.clock), r.mode, r.bits);
    }

    /// The control registers are written with SSE clear, so any change
    /// costs a disable/enable pair - paid only when something moved.
    static void reconfigure(SpiClock c, SpiMode m, SpiDataSize bits) {
        const uint32_t word = spi_control_word(c, m, bits);
        if (word == applied_word_) {
            return;
        }
        applied_word_ = word;
        applied_.mode = m;
        applied_.clock = c;
        applied_.bits = spi_data_bits(bits);
        S::enable(false);
        (void)S::configure(applied_);
        S::enable(true);
    }

    // ---- the frames ---------------------------------------------------------

    /// The next frame out of an advancing pointer: one byte, or two
    /// low-first for a half-word; 0xFFFF from a null one.
    template <bool wide>
    [[gnu::always_inline]] static uint32_t next_frame(const uint8_t*& p) {
        if (p == nullptr) {
            return 0xFFFFu;
        }
        uint32_t v = p[0];
        if constexpr (wide) {
            v |= static_cast<uint32_t>(p[1]) << 8;
            p += 2;
        } else {
            p += 1;
        }
        return v;
    }
    /// A frame into an advancing pointer, the same packing.
    template <bool wide>
    [[gnu::always_inline]] static void store_frame(uint8_t*& p, uint32_t v) {
        p[0] = static_cast<uint8_t>(v);
        if constexpr (wide) {
            p[1] = static_cast<uint8_t>(v >> 8);
            p += 2;
        } else {
            p += 1;
        }
    }

    /// The spin budget of a polled transaction: one slowest frame (65024
    /// x 16 reference cycles, about 200 000 turns of the wait) per frame
    /// and one more, saturated.
    static uint32_t spin_budget(uint32_t frames) {
        return frames < 21'000u ? 200'000u * (frames + 1u) : 0xFFFF'FFFFu;
    }

    /// THE POLLED LOOP, THE RECEIVE SHAPE (the file header): `count`
    /// frames out of `out` (null = 0xFFFF) and into `in` (null = thrown
    /// away as they land), a FIFO's worth IN FLIGHT - written and not yet
    /// read back. The transmit FIFO is
    /// empty on entry - every earlier transaction took back or drained
    /// every frame it wrote - so the first `fifo_depth` frames go in with
    /// no flag read (12.3.4.3's priming, with the port enabled); then per
    /// frame: wait for RNE, read, and write the frame a FIFO's depth
    /// ahead, which cannot find the transmit FIFO full because fewer
    /// than its depth are in flight. NO HANDLER CAN OVERRUN IT: whatever
    /// runs while this loop is held off, the frames that can land in the
    /// receive FIFO are the ones in flight, and those are never more than
    /// its depth - measured under a tick handler stretched past eight
    /// frame times, every frame exact and the overrun flag clear
    /// (docs/rp2350/spi.md). False when `budget` ran out waiting - the
    /// block stopped.
    template <bool wide>
    static bool run_duplex(const uint8_t* out, uint8_t* in, uint16_t count, uint32_t& budget) {
        Regs& regs = S::regs();
        const uint16_t ahead = count < S::fifo_depth ? count : S::fifo_depth;
        const uint8_t* wp = out;
        for (uint16_t k = 0; k < ahead; ++k) {
            regs.SSPDR = next_frame<wide>(wp);
        }
        uint8_t* rp = in;
        for (uint16_t k = count; k != 0u; --k) {
            while ((regs.SSPSR & SpiFlag::rx_not_empty) == 0u) {
                if (budget-- == 0u) {
                    return false;
                }
            }
            const uint32_t v = regs.SSPDR;
            if (in != nullptr) {
                store_frame<wide>(rp, v);
            }
            if (k > ahead) {
                regs.SSPDR = next_frame<wide>(wp);
            }
        }
        return true;
    }

    /// THE POLLED LOOP, THE TRANSMIT-ONLY SHAPE: `count` frames out of
    /// `out` (null = 0xFFFF) with the answers thrown away. It paces on
    /// TNF alone and reads nothing per frame (two accesses of the block a
    /// frame: SSPSR, SSPDR out), so the receive FIFO fills and OVERRUNS
    /// BY DESIGN: a frame pushed on a full receive FIFO is dropped and
    /// RORRIS raised while the block keeps shifting, which costs nothing
    /// while RORIM is masked - and this host never arms it. The tail
    /// waits for BSY to clear (the last frame shifted, the transmit FIFO
    /// empty), drains the receive FIFO and clears the overrun, so the
    /// block is left as every other path leaves it. THAT TAIL IS ITS
    /// PRICE: up to a FIFO's worth of reads after the wire has finished,
    /// which the receive shape spends under the wire instead - so this
    /// shape is taken only where the receive shape would fall behind the
    /// wire, which on both of the RP2350's cores is a divisor below four
    /// on 8-bit frames (letter e: 27 and 38 cycles a frame of loop against
    /// 16 of wire at clk_peri / 2; at / 4 and below both shapes read the
    /// wire's own figure), and only on a run longer than the FIFO, where
    /// the tail is shorter than what it saves.
    template <bool wide>
    static bool run_write(const uint8_t* out, uint16_t count, uint32_t& budget) {
        Regs& regs = S::regs();
        const uint8_t* wp = out;
        for (uint16_t k = count; k != 0u; --k) {
            while ((regs.SSPSR & SpiFlag::tx_not_full) == 0u) {
                if (budget-- == 0u) {
                    return false;
                }
            }
            regs.SSPDR = next_frame<wide>(wp);
        }
        while ((regs.SSPSR & SpiFlag::busy) != 0u) {
            if (budget-- == 0u) {
                return false;
            }
        }
        while ((regs.SSPSR & SpiFlag::rx_not_empty) != 0u) {
            (void)regs.SSPDR;
        }
        regs.SSPICR = SpiInterrupt::overrun;
        return true;
    }

    /// One phase of a polled request, by its shape and its width: the
    /// receive shape - discarding when there is no in buffer - unless the
    /// run is a write longer than the FIFO at a rate the receive shape
    /// cannot keep (run_write's comment: a divisor below four on 8-bit
    /// frames; a 16-bit frame at that rate is twice as long and the
    /// receive shape keeps it).
    static bool run_polled(const uint8_t* out, uint8_t* in, uint16_t count, SpiDataSize bits,
                           uint32_t& budget) {
        if (spi_frame_is_halfword(bits)) {
            return run_duplex<true>(out, in, count, budget);
        }
        if (in == nullptr && count > S::fifo_depth && applied_.clock.divisor() < 4u) {
            return run_write<false>(out, count, budget);
        }
        return run_duplex<false>(out, in, count, budget);
    }

    /// After a polled wait ran out: the block is left as it is (the
    /// arbiter's recover() is the way back), the frames still in the
    /// FIFOs let out and thrown away, the status says what happened.
    static void settle_fault() {
        status_ = spi_dma_fault;
        uint32_t spins = 200'000u;
        while (S::busy() && spins-- != 0u) {
        }
        S::flush_rx();
    }

    /// The pump's write side: frames while fewer than a FIFO's worth are
    /// in flight, up to the current phase's end - no TNF read, for the
    /// reason run_polled gives.
    static void fill() {
        Tenure& t = tenure_;
        Regs& regs = S::regs();
        uint16_t sent = t.sent;
        const uint16_t end = t.phase_end;
        const uint16_t received = t.received;
        const uint8_t* out = t.out;
        if (spi_frame_is_halfword(t.req.bits)) {
            while (sent < end && static_cast<uint16_t>(sent - received) < S::fifo_depth) {
                regs.SSPDR = next_frame<true>(out);
                ++sent;
            }
        } else {
            while (sent < end && static_cast<uint16_t>(sent - received) < S::fifo_depth) {
                regs.SSPDR = next_frame<false>(out);
                ++sent;
            }
        }
        t.out = out;
        t.sent = sent;
    }

    /// The pump's read side: what the receive FIFO holds - command echoes
    /// discarded, data stored - and the phase boundary crossed when the
    /// last command frame is in (D/C high, the data buffer next).
    static void take() {
        Tenure& t = tenure_;
        Regs& regs = S::regs();
        uint16_t received = t.received;
        uint8_t* in = t.in;
        const uint16_t data_from = t.req.cmd_len;
        const bool wide = spi_frame_is_halfword(t.req.bits);
        while ((regs.SSPSR & SpiFlag::rx_not_empty) != 0u) {
            const uint32_t v = regs.SSPDR;
            if (received >= data_from && in != nullptr) {
                if (wide) {
                    store_frame<true>(in, v);
                } else {
                    store_frame<false>(in, v);
                }
            }
            ++received;
        }
        t.in = in;
        t.received = received;
        if (t.in_cmd && received >= data_from) {
            t.in_cmd = false;
            t.req.dc.set();
            t.out = t.req.tx.get();
            t.phase_end = t.total;
        }
    }

    static inline Tenure tenure_{};
    static inline uint8_t status_ = spi_ok;
    static inline volatile bool dma_done_ = false;
    static inline volatile bool dma_active_ = false;
    // One cell each serves both beats: a byte access at a half-word's
    // address is its low byte on these little-endian cores.
    static inline uint16_t rx_sink_ = 0;
    static constexpr uint16_t tx_dummy_ = 0xFFFFu;
    static inline SpiConfig applied_{};
    static inline uint32_t applied_word_ = 0;
    static inline uint32_t pclk_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<SpiClock> ceiling_{};
    static inline typename Chip::DelayRate cs_rate_{};
};

// =============================================================================
// The client task
// =============================================================================

/**
 * Pl022Client<Chip, n, pins>
 *
 * The other end of the wire: SCK, TX-in and the select are inputs, the
 * host sets the pace, and the only thing this side controls is WHAT it
 * has ready to shift out when the next clock arrives - up to
 * `frames_ahead` frames in the transmit FIFO, written on TNF. A client
 * that falls behind does not stall the bus: what the host reads is
 * whatever the shifter held, a wrong answer and never a missing one. THE
 * SELECT IS THE PAD: the PL022 in slave mode frames on SSPFSSIN, so
 * `pins.cs` goes to the peripheral and `selected()` reads the same pad.
 * `drive_output(false)` leaves the transmit line undriven, a DARK
 * LISTENER on a shared harness - by releasing the PAD, since what SOD
 * does to a pad is the chip's fact and not the block's (the file header).
 * The family's own header aliases this to `SpiClient<n, pins>`.
 */
template <Pl022Chip Chip, uint8_t n, typename Chip::Pins pins>
class Pl022Client {
    using S = Pl022Ssp<Chip, n>;
    static_assert(Chip::pins_valid(n, pins),
                  "brio SpiClient: these pads cannot carry this instance's signals - see the "
                  "family's own pin table");
    static constexpr uint8_t sck_pin = Chip::sck_pad(pins);
    static constexpr uint8_t tx_pin = Chip::tx_pad(pins);
    static constexpr uint8_t rx_pin = Chip::rx_pad(pins);
    static constexpr uint8_t cs_pin = Chip::cs_pad(pins);
    static_assert(rx_pin != Chip::no_pad && cs_pin != Chip::no_pad,
                  "brio SpiClient: a client listens on RX and frames on its CS pad");
    using SckPad = typename Chip::template Pad<sck_pin>;
    using RxPad = typename Chip::template Pad<rx_pin>;
    using CsPad = typename Chip::template Pad<cs_pin>;
    using TxPad = typename Chip::template Pad<tx_pin != Chip::no_pad ? tx_pin : sck_pin>;

public:
    Pl022Client() = delete;
    using Resource = S;
    static constexpr typename Chip::Pins pin_set = pins;

    struct Config {
        SpiMode mode = SpiMode::mode0;
        SpiFormat format = SpiFormat::motorola;
        uint8_t bits = 8;             ///< 4..16
        /// Whether the transmit line is driven at all (SOD clear).
        bool drive_output = true;
    };

    /// Bring the instance up as a client. `clock` is the app's Clock tag:
    /// SSPCLK must run at least twelve times the host's clock, which is
    /// the caller's to know. False when the block did not come up or the
    /// configuration is refused.
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        Chip::Interrupts::disable(S::irq());
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
        SckPad::function(Chip::pad_function, Chip::sck_pad_config());
        RxPad::function(Chip::pad_function, Chip::client_rx_pad_config());
        CsPad::function(Chip::pad_function, Chip::cs_pad_config());
        drive_output(cfg.drive_output);
        S::flush_rx();
        Chip::Interrupts::enable(S::irq());
        return true;
    }

    /// The answer line handed to the peripheral, or taken back as an
    /// undriven input - the dark listener (the file header: the PAD is
    /// the lever, because SOD's effect on one is the chip's fact).
    static void drive_output(bool on) {
        if constexpr (tx_pin != Chip::no_pad) {
            if (on) { TxPad::function(Chip::pad_function, Chip::tx_pad_config()); } else { TxPad::release(); }
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
    static bool selected() { return !CsPad::read(); }

    static bool overrun() { return (S::raw_pending() & SpiInterrupt::overrun) != 0u; }
    static void clear_overrun() { S::clear_pending(SpiInterrupt::overrun); }
    static uint32_t flags() { return S::flags(); }

    /// The ISR body: the raised-and-enabled sources, for the app's glue.
    [[gnu::always_inline]] static uint32_t isr() { return S::isr(); }
    static void interrupts(uint32_t mask, bool on) { S::interrupts(mask, on); }

    static void release() {
        Chip::Interrupts::disable(S::irq());
        S::interrupts(SpiInterrupt::all, false);
        S::enable(false);
        SckPad::release();
        RxPad::release();
        CsPad::release();
        if constexpr (tx_pin != Chip::no_pad) {
            TxPad::release();
        }
        S::hold();
    }

private:
    static inline uint8_t bits_ = 8;
};

} // namespace brio
