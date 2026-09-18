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
 * THE PUMP keeps up to `fifo_depth` frames in flight: on every receive
 * interrupt (the FIFO's half, or the timeout for a tail) it takes what
 * came back and writes as many more as it took. THE TRANSMIT INTERRUPT
 * IS NEVER THE PUMP'S EDGE - a frame that came BACK is the one witness
 * the bus is idle for the next, the other strata's rule - and the pump
 * writes only while fewer than a FIFO's worth are in flight, so the
 * receive FIFO cannot overrun. The phase boundary drains: the first data
 * frame goes out only when the last command frame has come back, so D/C
 * flips between them on the wire. Two completion styles, the request's
 * choice: `polled` false runs on the interrupt with the kernel free
 * between batches, `polled` true spins inside start() with this
 * instance's own line silenced.
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
 * the pump. A request in 16-bit frames falls back to the pump: the
 * engines carry bytes. FRAMES IN A BYTE BUFFER: one byte per 8-bit
 * frame, two bytes low-first per 16-bit frame - the other strata's rule.
 *
 * NOT built, each with its reason in docs/pl022/README.md: the block's
 * own frame signal as a host select, the National Semiconductor
 * Microwire transaction (half duplex, a control frame and then the
 * slave's answer, where this engine's Request is full duplex and one
 * frame wide), the client on the DMA engines.
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <optional>
#include <type_traits>

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
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio SpiHost: the DMA engines must carry uint8_t elements - the Request's buffers are "
         "bytes and the engined path serves 8-bit frames");

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
        typename Chip::PinRef cs;   ///< asserted low around the transaction
        typename Chip::PinRef dc;   ///< display D/C line; null = no such pin
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
        // Polled pump: silence this instance's own line (a bound handler
        // would steal the frames). Global interrupts STAY ENABLED.
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
        Chip::Interrupts::disable(S::irq());
        in_cmd_ = false;
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
    /// slowest frame is 65024 x 16 reference cycles; the budget scales).
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

    /// Write frames while the transmit FIFO takes them and fewer than a
    /// FIFO's worth are in flight; the data phase waits for the command
    /// phase to come back whole (the D/C flip on the wire).
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
