/*
 * spi.hpp
 *
 * The SPI / I2S block (RM0444 ch. 35) in the two strata every brio bus
 * driver has (docs/design/spi-bus.md):
 *
 *   Spi<n>        the RESOURCE: which instance, where its registers are,
 *                 its APB clock and reset, its NVIC line, its DMAMUX
 *                 request pair, the whole register description of the
 *                 chapter in both of its personalities (SPI Motorola,
 *                 SPI TI, and the I2S the same registers become when
 *                 I2SMOD is set) and the two DISCIPLINES the chapter
 *                 states: configuration only with SPE clear, and the
 *                 disable PROCEDURE of 35.5.9;
 *   SpiHost<...>  the TASK util/spi_bus.hpp drives: the transfer engine
 *                 with the SAME Request shape and the same public
 *                 surface as avrdx/spi.hpp's and samc21/spi.hpp's, so a
 *                 device client compiles on the third architecture
 *                 untouched;
 *   SpiClient<..> the other end of the wire: the hardware NSS input, the
 *                 preload and the one-ahead pump, a polled surface and
 *                 an ISR body. A client is a PROTOCOL and the protocol
 *                 is the application's, so this half decides nothing.
 *
 * SCOPE. The chapter whole, bar what docs/stm32g0/spi.md declines with a
 * reason: both roles, all four modes, both bit orders, the eight baud
 * prescalers, data sizes 4..16 with the FRXTH threshold and the
 * byte-versus-half-word access rule, every NSS arrangement (software,
 * hardware input, hardware output, NSS pulse mode), TI frame format,
 * the hardware CRC in both lengths, half-duplex (BIDIMODE) and
 * receive-only simplex, the FIFO levels, the error flags with their
 * clear sequences, the DMA enables with the packing bits, and the I2S
 * mode with its four standards, its three data lengths, its prescaler
 * and its master clock output.
 *
 * FACTS THAT SHAPE THE CODE
 *
 *  - 35.5.7 is a WITH-SPE-CLEAR procedure: "Configuration of SPI" writes
 *    CR1 and CR2 with the peripheral disabled, and four fields say so in
 *    their own register description as well (CRCEN, CRCL, FRF, NSSP,
 *    plus LDMA_TX/LDMA_RX "has to be written when the SPI is disabled").
 *    Another four are gated by the softer "should not be changed when
 *    communication is ongoing" (BR, MSTR, LSBFIRST, CPOL/CPHA - and
 *    35.5.6's own Note hardens the last pair into "the SPI must be
 *    disabled by resetting the SPE bit"). THIS DRIVER REFUSES ALL OF
 *    THEM WHILE SPE IS SET - one rule instead of two, so that no caller
 *    has to remember which sentence a field got. Which of those the
 *    SILICON really enforces is a different question and the bench asks
 *    it directly (test_stm32_spi letter a writes each field with SPE set
 *    and reports what stuck). The verbs that stay open under a running
 *    SPI are the ones that MUST be: SSI (the software select, whose
 *    whole use is to be moved under a live master), CRCNEXT (35.9.1:
 *    "written as soon as the last data is written in SPIx_DR"), the
 *    three interrupt enables, and the data register.
 *
 *  - THERE IS NO RAW SPE CLEAR IN THIS FILE. disable() is 35.5.9's
 *    procedure - wait FTLVL = 00, wait BSY = 0, clear SPE, drain the
 *    RXFIFO until FRLVL = 00 - and it is the only verb that turns the
 *    peripheral off. That makes ES0548 2.12.1's workaround structural:
 *    the erratum's three cases are a master transmitter disabled with
 *    the data register full, a receive-only master, and a slave whose
 *    NSS was removed mid-transfer, and the first is exactly what
 *    "FTLVL = 00 then BSY = 0" excludes. The receive-only case has its
 *    own procedure and its own verb (disable_receive_only()), whose
 *    contract is the erratum's own advice: ignore BSY there.
 *
 *  - ES0548 2.12.2 says BSY may sporadically stay high at the end of a
 *    SLAVE transfer. So nothing on the client side of this file waits on
 *    BSY: a received frame is RXNE, and the end of a transaction is the
 *    NSS pad going high. Every wait in the file is bounded anyway - a
 *    bus whose clock never runs must not hang the kernel - and reports
 *    false rather than spinning.
 *
 *  - RXNE AND NOT TXE DRIVES THE HOST PUMP, the samc21 lesson in this
 *    family's clothes. TXE means "the transmit FIFO has room", which is
 *    true well before a frame is on the wire and true again immediately
 *    after; RXNE means "a frame has been shifted in", which on a
 *    full-duplex bus is exactly "one frame moved, both ways" - and
 *    reading DR is both the capture and the acknowledgement. One
 *    interrupt per frame, nothing to disambiguate, and the transaction
 *    ends when the LAST frame has come back rather than when the last
 *    one was handed to the shifter.
 *
 *  - THE FIFO ACCESS WIDTH IS PART OF THE FRAME SIZE (35.5.9, figure
 *    363): with DS <= 8 a frame is right-aligned in a BYTE and DR must
 *    be accessed a byte at a time, or the silicon PACKS two frames into
 *    one 16-bit access; with DS > 8 a frame is right-aligned in a
 *    half-word and DR is accessed 16 bits at a time. FRXTH must agree
 *    with the read width (1 = RXNE at a quarter of the FIFO, i.e. one
 *    byte; 0 = RXNE at a half, i.e. one half-word). Every read and write
 *    verb here therefore comes in a byte flavour and a half-word one,
 *    and data(size, v) picks by the size the caller states.
 *
 *  - THE CHIP SELECT A TRANSACTION IS FRAMED BY IS A GPIO, as on both
 *    other targets, and the reason is 35.5.5: hardware NSS output falls
 *    "as soon as the SPI is enabled in master mode (SPE = 1)" and rises
 *    when it is disabled - it frames the peripheral's LIFETIME and not a
 *    transaction - while NSS pulse mode (35.5.12) frames a DATA FRAME,
 *    raising NSS between consecutive frames. Neither is what a device
 *    wants around a command plus its data. Both are resource facts the
 *    bench measures; the task's Request carries a PinRef.
 *
 *  - THE VECTOR IS SHARED where the part has a SPI3: SPI2 and SPI3 sit
 *    on SPI2_3_IRQn (the reserve derives it from SPI3_BASE), so an app
 *    binds BRIO_STM32G0_SPI2_HANDLER and calls both instances' bodies.
 *
 *  - THERE IS NO KERNEL-CLOCK MULTIPLEXER FOR SPI. The baud prescaler
 *    divides PCLK and nothing else (35.9.1's BR field says fPCLK), which
 *    is why init() takes the app's Clock tag and rebase() is a pure
 *    re-resolution of the ceiling. The I2S half DOES have one - I2S1SEL
 *    / I2S2SEL, in CCIPR on the small parts and CCIPR2 on the G0Bx/G0Cx
 *    - and it is a resource verb of its own (i2s_kernel_clock()).
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "stm32g0xx.h"

#include "stm32g0/clock.hpp"
#include "stm32g0/delay.hpp"
#include "stm32g0/device_tables.hpp"
#include "stm32g0/dma_engine.hpp"
#include "stm32g0/nvic.hpp"
#include "stm32g0/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// =============================================================================
// The vocabulary (35.9.1, 35.9.2)
// =============================================================================

/// Which end of the bus this instance is (CR1.MSTR). There IS a runtime
/// demotion on this peripheral, unlike the SAM's: a master whose NSS
/// input goes low raises MODF and the silicon clears MSTR and SPE for
/// it (35.5.11) - the AVR's behaviour, met again. `Spi<n>::mode_fault()`
/// reports it and clear_mode_fault() is the chapter's own sequence.
enum class SpiRole : uint8_t { host = 1, client = 0 };

/// The four clock phase/polarity combinations. SPELLED THE SAME WAY AS
/// avrdx/spi.hpp and samc21/spi.hpp - bit 1 is CPOL, bit 0 is CPHA - so
/// brio's SpiMode is one vocabulary across three architectures even
/// though here the two bits sit at the BOTTOM of CR1 and there they were
/// two bits of CTRLA and a field of CTRLB.
enum class SpiMode : uint8_t {
    mode0 = 0,   ///< SCK idle low,  first edge captures
    mode1 = 1,   ///< SCK idle low,  second edge captures
    mode2 = 2,   ///< SCK idle high, first edge captures
    mode3 = 3,   ///< SCK idle high, second edge captures
};

constexpr bool spi_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 0x02u) != 0; }
constexpr bool spi_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 0x01u) != 0; }

/// CR1.BR[2:0], all eight codes. The name is the DIVISION, which is what
/// a device's datasheet limit is spoken in and what avrdx/spi.hpp's
/// SpiClock names too - the AVR's ladder stops at div128 because that is
/// where its prescaler stops; this one has a div256 as well.
enum class SpiClock : uint8_t {
    div2 = 0, div4 = 1, div8 = 2, div16 = 3,
    div32 = 4, div64 = 5, div128 = 6, div256 = 7,
};

constexpr uint16_t spi_division(SpiClock c) {
    return static_cast<uint16_t>(2u << static_cast<uint8_t>(c));
}

/// What SCK a code really produces at this PCLK.
constexpr uint32_t spi_sck_hz(uint32_t pclk_hz, SpiClock c) {
    return pclk_hz / spi_division(c);
}

constexpr uint32_t spi_max_sck_hz(uint32_t pclk_hz) { return pclk_hz / 2u; }
constexpr uint32_t spi_min_sck_hz(uint32_t pclk_hz) { return pclk_hz / 256u; }

/**
 * The FASTEST code at or below `max_sck_hz` - the chooser a device's
 * datasheet limit is spoken to (avrdx/spi.hpp's spi_clock_for(), same
 * shape and same contract). Nullopt when even PCLK/256 is too fast:
 * a ceiling this generator cannot honour is REFUSED and never rounded
 * up, because a bus run faster than a device's limit is a fault the
 * caller must see.
 */
constexpr std::optional<SpiClock> spi_rate_for(uint32_t pclk_hz, uint32_t max_sck_hz) {
    if (pclk_hz == 0u || max_sck_hz == 0u) {
        return {};
    }
    for (uint8_t i = 0; i <= static_cast<uint8_t>(SpiClock::div256); ++i) {
        const SpiClock c = static_cast<SpiClock>(i);
        if (spi_sck_hz(pclk_hz, c) <= max_sck_hz) {
            return c;
        }
    }
    return {};
}

/**
 * CR2.DS[3:0], the frame size in bits. The codes below 0011 are "Not
 * used" and 35.9.2 says the silicon FORCES them to 0111 (8-bit) rather
 * than ignoring the write - which is why this enum names only the twelve
 * real ones and spi_data_bits_valid() refuses the rest instead of
 * trusting a register that will lie about what it holds.
 */
enum class SpiDataSize : uint8_t {
    bits4 = 3, bits5 = 4, bits6 = 5, bits7 = 6, bits8 = 7,
    bits9 = 8, bits10 = 9, bits11 = 10, bits12 = 11,
    bits13 = 12, bits14 = 13, bits15 = 14, bits16 = 15,
};

constexpr uint8_t spi_data_bits(SpiDataSize d) {
    return static_cast<uint8_t>(static_cast<uint8_t>(d) + 1u);
}
constexpr bool spi_data_size_valid(SpiDataSize d) {
    return static_cast<uint8_t>(d) >= 3u && static_cast<uint8_t>(d) <= 15u;
}
/// SpiDataSize from a bit count, 4..16. Nullopt outside that range - the
/// chapter's own "the minimum data length is 4 bits".
constexpr std::optional<SpiDataSize> spi_data_size_of(uint8_t bits) {
    if (bits < 4u || bits > 16u) {
        return {};
    }
    return static_cast<SpiDataSize>(bits - 1u);
}
/// Whether a frame of this size occupies a HALF-WORD of the FIFO
/// (35.5.9's figure 363): 4..8 bits ride in a byte, 9..16 in a
/// half-word, and the access width must follow - see the file comment.
constexpr bool spi_frame_is_halfword(SpiDataSize d) { return spi_data_bits(d) > 8u; }
/// The mask of the bits a frame of this size really carries, so a
/// caller (and this file's own loops) can right-align a datum without
/// re-deriving the arithmetic.
constexpr uint16_t spi_frame_mask(SpiDataSize d) {
    return static_cast<uint16_t>((1u << spi_data_bits(d)) - 1u);
}

/// CR2.FRXTH, the RXFIFO threshold RXNE is raised at. It is not a free
/// choice: 35.5.7 step 3e makes it the read ACCESS SIZE, so it follows
/// the data size unless a caller is deliberately doing packed 8-bit
/// traffic through 16-bit accesses.
enum class SpiRxThreshold : uint8_t {
    half = 0,     ///< RXNE at 1/2 the FIFO: a 16-bit read
    quarter = 1,  ///< RXNE at 1/4 the FIFO: an 8-bit read
};

/// CR2.FRF: which protocol the frame is shaped by. TI mode forces the
/// clock polarity and phase and takes NSS away from SSM/SSI/SSOE
/// altogether (35.5.13).
enum class SpiFrameFormat : uint8_t { motorola = 0, ti = 1 };

/// How the two sides of CR1's NSS logic are set up (35.5.5, figure 361).
/// Not a register field: a reading of three bits (SSM, SSI, SSOE) that
/// makes the four legal arrangements nameable.
enum class SpiNss : uint8_t {
    /// SSM = 1: the pad is free, the internal select is SSI. A master
    /// drives SSI high (or MODF fires); a client drives it low to select
    /// itself. This is the arrangement the transfer engine uses.
    software = 0,
    /// SSM = 0, SSOE = 0: the pad is an INPUT. On a client it is the
    /// chip select; on a master it is the multi-master monitor whose low
    /// level raises MODF.
    hardware_input = 1,
    /// SSM = 0, SSOE = 1: master only, the pad is an OUTPUT that falls
    /// at SPE and rises when the peripheral is disabled.
    hardware_output = 2,
    /// hardware_output plus CR2.NSSP: a pulse between consecutive
    /// frames (35.5.12; master, Motorola, CPHA = 0 only).
    hardware_pulse = 3,
};

constexpr bool spi_nss_is_output(SpiNss n) {
    return n == SpiNss::hardware_output || n == SpiNss::hardware_pulse;
}
constexpr bool spi_nss_uses_pad(SpiNss n) { return n != SpiNss::software; }

/// CR1.BIDIMODE / BIDIOE / RXONLY as one reading: which wires a
/// transfer really uses (35.9.1). `full_duplex` is the reset state.
enum class SpiDirection : uint8_t {
    full_duplex = 0,        ///< BIDIMODE 0, RXONLY 0 - two data lines
    receive_only = 1,       ///< BIDIMODE 0, RXONLY 1 - simplex in
    bidi_receive = 2,       ///< BIDIMODE 1, BIDIOE 0 - one line, in
    bidi_transmit = 3,      ///< BIDIMODE 1, BIDIOE 1 - one line, out
};

constexpr bool spi_direction_is_bidi(SpiDirection d) {
    return d == SpiDirection::bidi_receive || d == SpiDirection::bidi_transmit;
}

/// CR1.CRCL: the length of the hardware checksum, independent of the
/// frame size (35.5.14 gives CRC8 or CRC16 for 8-bit and 16-bit frames
/// and NO CRC at all for every other frame size).
enum class SpiCrcLength : uint8_t { crc8 = 0, crc16 = 1 };

/// SPIx_SR, by name. Read-only but for CRCERR, which is rc_w0.
struct SpiFlag {
    static constexpr uint32_t rxne = SPI_SR_RXNE;
    static constexpr uint32_t txe = SPI_SR_TXE;
    static constexpr uint32_t chside = SPI_SR_CHSIDE;
    static constexpr uint32_t underrun = SPI_SR_UDR;
    static constexpr uint32_t crc_error = SPI_SR_CRCERR;
    static constexpr uint32_t mode_fault = SPI_SR_MODF;
    static constexpr uint32_t overrun = SPI_SR_OVR;
    static constexpr uint32_t busy = SPI_SR_BSY;
    static constexpr uint32_t frame_error = SPI_SR_FRE;
    /// Every error 35.6's table 206 puts behind ERRIE.
    static constexpr uint32_t errors = crc_error | mode_fault | overrun | frame_error | underrun;
};

/// CR2's three interrupt enables (table 206). ERRIE covers CRCERR,
/// MODF, OVR and FRE in SPI mode and UDR, OVR and FRE in I2S mode -
/// one enable, six conditions, which is why isr() hands the caller the
/// masked status rather than a decision.
struct SpiInterrupt {
    static constexpr uint32_t txe = SPI_CR2_TXEIE;
    static constexpr uint32_t rxne = SPI_CR2_RXNEIE;
    static constexpr uint32_t error = SPI_CR2_ERRIE;
    static constexpr uint32_t all = txe | rxne | error;
};

// =============================================================================
// Pads
// =============================================================================

/**
 * Where the four SPI signals sit, with the AF the DATASHEET gives each
 * signal on each pad (DS13560 tables 13..19). The same shape as
 * usart.hpp's UartPins, and the same standing caveat: NO HEADER SYMBOL
 * CAN CHECK AN ALTERNATE FUNCTION NUMBER. What this file checks is that
 * every pad named is a real one on this device (PinSel::valid() reads
 * the reserve's port table) and that no two of them are the same pad;
 * that the AF really carries SPIn's signal on that pad is the
 * datasheet's claim and the bench's proof.
 *
 * `nss` may be left default-constructed: a null NSS pad is the software
 * arrangement (SSM = 1), which is what the transfer engine uses and what
 * every host that frames its transactions with an ordinary GPIO wants.
 * The three data pads are all optional in the same weak sense - a
 * transmit-only host needs no MISO, a receive-only one no MOSI - and
 * each task below asserts the ones ITS role cannot do without.
 */
struct SpiPins {
    PinSel sck;
    PinSel miso;
    PinSel mosi;
    PinSel nss;

    constexpr bool has_nss() const { return nss.valid(); }
};

/// Two pads are the same pad.
constexpr bool spi_pins_collide(const PinSel& a, const PinSel& b) {
    return a.valid() && b.valid() && a.port == b.port && a.pin == b.pin;
}

/// SCK must exist (there is no SPI without a clock line) and no two
/// named pads may be the same one.
constexpr bool spi_pins_valid(const SpiPins& p) {
    if (!p.sck.valid()) {
        return false;
    }
    const PinSel all[4] = {p.sck, p.miso, p.mosi, p.nss};
    for (uint8_t i = 0; i < 4u; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < 4u; ++j) {
            if (spi_pins_collide(all[i], all[j])) {
                return false;
            }
        }
    }
    return true;
}

// =============================================================================
// The configuration
// =============================================================================

/**
 * Everything CR1 and CR2 hold, as one constexpr struct - the thing
 * configure() writes wholesale with SPE clear, and the thing the tasks
 * below cache so that a per-request change costs a disable/enable pair
 * only when something really moved.
 */
struct SpiConfig {
    SpiRole role = SpiRole::host;
    SpiMode mode = SpiMode::mode0;
    SpiClock clock = SpiClock::div16;      ///< host only: BR is ignored by a client
    SpiDataSize bits = SpiDataSize::bits8;
    bool lsb_first = false;
    SpiNss nss = SpiNss::software;
    SpiDirection direction = SpiDirection::full_duplex;
    SpiFrameFormat format = SpiFrameFormat::motorola;

    /// FRXTH. `automatic` is not a code: leave it as the default and
    /// configure() derives it from `bits` (35.5.7 step 3e), which is
    /// what every unpacked transfer wants. State it only for packed
    /// 8-bit traffic through 16-bit accesses.
    std::optional<SpiRxThreshold> rx_threshold{};

    /// 35.5.14. `polynomial` must be ODD (the chapter says so twice and
    /// the silicon does not check); 0x0007 is the reset value.
    bool crc = false;
    SpiCrcLength crc_length = SpiCrcLength::crc8;
    uint16_t crc_polynomial = 0x0007u;

    /// CR2's DMA request enables and the packing bits that go with them
    /// (35.9.2: LDMA_* matter only with the corresponding DMAEN set and
    /// a 16-bit access to an 8-bit frame).
    bool dma_transmit = false;
    bool dma_receive = false;
    bool last_dma_transmit_odd = false;
    bool last_dma_receive_odd = false;

    /// The internal select a software-NSS instance presents to itself.
    /// A MASTER must keep this HIGH or MODF fires the moment SPE rises;
    /// a client selects itself with it low. configure() therefore
    /// defaults it FROM THE ROLE when it is not stated.
    std::optional<bool> software_select{};
};

/// The SSI a configuration really applies: the caller's when stated,
/// otherwise "a master selects nothing, a client selects itself".
constexpr bool spi_software_select(const SpiConfig& c) {
    return c.software_select.value_or(c.role == SpiRole::host);
}

/// The FRXTH a configuration really applies: the caller's when stated,
/// otherwise the one the frame size implies.
constexpr SpiRxThreshold spi_rx_threshold(const SpiConfig& c) {
    if (c.rx_threshold) {
        return *c.rx_threshold;
    }
    return spi_frame_is_halfword(c.bits) ? SpiRxThreshold::half : SpiRxThreshold::quarter;
}

/**
 * The rules the chapter states and the registers do not enforce. Each
 * clause is one sentence of RM0444 ch. 35, named in the comment, so a
 * refusal can be traced to the silicon's own claim rather than to this
 * driver's taste.
 */
constexpr bool spi_config_valid(const SpiConfig& c) {
    if (!spi_data_size_valid(c.bits)) {
        return false;   // 35.9.2: a "Not used" code is FORCED to 8-bit
    }
    if (c.crc) {
        // 35.5.14: "The SPI offers CRC8 or CRC16 calculation
        // independently of the frame data length, which can be fixed to
        // 8-bit or 16-bit. For all the other data frame lengths, no CRC
        // is available."
        if (c.bits != SpiDataSize::bits8 && c.bits != SpiDataSize::bits16) {
            return false;
        }
        if ((c.crc_polynomial & 1u) == 0u) {
            return false;   // 35.9.5: "The polynomial value should be odd only"
        }
    }
    if (c.format == SpiFrameFormat::ti) {
        // 35.5.13: TI mode "makes the configuration of NSS management
        // through the SPIx_CR1 and SPIx_CR2 registers (SSM, SSI, SSOE)
        // impossible", and 35.9.2's NSSP note says NSSP "has no meaning
        // if CPHA = 1, or FRF = 1".
        if (c.nss == SpiNss::hardware_pulse) {
            return false;
        }
    }
    if (c.nss == SpiNss::hardware_pulse) {
        // 35.5.12: NSS pulse mode "takes effect only if the SPI
        // interface is configured as Motorola SPI master (FRF = 0) with
        // capture on the first edge (CPHA = 0)".
        if (c.role != SpiRole::host || spi_cpha(c.mode)) {
            return false;
        }
    }
    if (c.nss == SpiNss::hardware_output && c.role != SpiRole::host) {
        return false;   // 35.5.5: "this configuration is only used when the MCU is master"
    }
    if (c.last_dma_transmit_odd && !c.dma_transmit) {
        return false;   // 35.9.2: LDMA_TX "has significance only if TXDMAEN is set"
    }
    if (c.last_dma_receive_odd && !c.dma_receive) {
        return false;
    }
    if ((c.last_dma_transmit_odd || c.last_dma_receive_odd) && spi_frame_is_halfword(c.bits)) {
        return false;   // ...and only "if packing mode is used (data length <= 8-bit)"
    }
    return true;
}

/// CR1 for a configuration, SPE clear. The enable is a separate store,
/// deliberately: 35.5.8 wants the client up and its first datum loaded
/// before the master's clock arrives, so the two acts are two verbs.
constexpr uint32_t spi_cr1(const SpiConfig& c) {
    uint32_t v = 0;
    if (spi_direction_is_bidi(c.direction)) {
        v |= SPI_CR1_BIDIMODE;
        if (c.direction == SpiDirection::bidi_transmit) {
            v |= SPI_CR1_BIDIOE;
        }
    } else if (c.direction == SpiDirection::receive_only) {
        v |= SPI_CR1_RXONLY;
    }
    if (c.crc) {
        v |= SPI_CR1_CRCEN;
        if (c.crc_length == SpiCrcLength::crc16) {
            v |= SPI_CR1_CRCL;
        }
    }
    if (c.nss == SpiNss::software) {
        v |= SPI_CR1_SSM;
    }
    if (spi_software_select(c)) {
        v |= SPI_CR1_SSI;
    }
    if (c.lsb_first) {
        v |= SPI_CR1_LSBFIRST;
    }
    v |= static_cast<uint32_t>(c.clock) << SPI_CR1_BR_Pos;
    if (c.role == SpiRole::host) {
        v |= SPI_CR1_MSTR;
    }
    if (spi_cpol(c.mode)) {
        v |= SPI_CR1_CPOL;
    }
    if (spi_cpha(c.mode)) {
        v |= SPI_CR1_CPHA;
    }
    return v;
}

/// CR2 for a configuration, interrupt enables OFF (they are the task's
/// to arm and the ISR's to drop).
constexpr uint32_t spi_cr2(const SpiConfig& c) {
    uint32_t v = static_cast<uint32_t>(c.bits) << SPI_CR2_DS_Pos;
    if (spi_rx_threshold(c) == SpiRxThreshold::quarter) {
        v |= SPI_CR2_FRXTH;
    }
    if (c.last_dma_transmit_odd) {
        v |= SPI_CR2_LDMATX;
    }
    if (c.last_dma_receive_odd) {
        v |= SPI_CR2_LDMARX;
    }
    if (c.format == SpiFrameFormat::ti) {
        v |= SPI_CR2_FRF;
    }
    if (c.nss == SpiNss::hardware_pulse) {
        v |= SPI_CR2_NSSP;
    }
    if (spi_nss_is_output(c.nss)) {
        v |= SPI_CR2_SSOE;
    }
    if (c.dma_transmit) {
        v |= SPI_CR2_TXDMAEN;
    }
    if (c.dma_receive) {
        v |= SPI_CR2_RXDMAEN;
    }
    return v;
}

// =============================================================================
// The I2S half of the block (35.7, 35.9.8, 35.9.9)
// =============================================================================

/// I2SCFGR.I2SSTD - the four audio protocols of 35.7.2.
enum class I2sStandard : uint8_t {
    philips = 0,        ///< WS falls one bit before the MSB
    msb_justified = 1,  ///< left justified
    lsb_justified = 2,  ///< right justified
    pcm = 3,            ///< PCMSYNC then chooses short or long frame sync
};

/// I2SCFGR.DATLEN. Code 11 is "Not allowed" and is not spelled.
enum class I2sDataLength : uint8_t { bits16 = 0, bits24 = 1, bits32 = 2 };

/// I2SCFGR.CHLEN, the channel frame. It is only a choice with
/// DATLEN = 16-bit; for 24 and 32 the silicon fixes it at 32 whatever is
/// written (35.9.8's own note), which is why i2s_channel_bits() below
/// answers with the EFFECTIVE width and not with the bit.
enum class I2sChannelLength : uint8_t { bits16 = 0, bits32 = 1 };

/// I2SCFGR.I2SCFG - which side of the wire, and which direction. An I2S
/// is half duplex by construction (35.3), so the direction is part of
/// the mode and not a separate flag.
enum class I2sMode : uint8_t {
    slave_transmit = 0,
    slave_receive = 1,
    master_transmit = 2,
    master_receive = 3,
};

constexpr bool i2s_is_master(I2sMode m) {
    return m == I2sMode::master_transmit || m == I2sMode::master_receive;
}
constexpr bool i2s_is_transmit(I2sMode m) {
    return m == I2sMode::slave_transmit || m == I2sMode::master_transmit;
}

/// The channel frame really in force: 32 bits for any data length above
/// 16 (35.9.8's CHLEN note), the caller's choice otherwise.
constexpr uint8_t i2s_channel_bits(I2sDataLength d, I2sChannelLength c) {
    if (d != I2sDataLength::bits16) {
        return 32u;
    }
    return c == I2sChannelLength::bits32 ? 32u : 16u;
}

constexpr uint8_t i2s_data_bits(I2sDataLength d) {
    switch (d) {
        case I2sDataLength::bits16: return 16u;
        case I2sDataLength::bits24: return 24u;
        default: return 32u;
    }
}

struct I2sConfig {
    I2sMode mode = I2sMode::master_transmit;
    I2sStandard standard = I2sStandard::philips;
    I2sDataLength data_length = I2sDataLength::bits16;
    I2sChannelLength channel_length = I2sChannelLength::bits16;
    /// PCMSYNC - long frame synchronization. Meaningful only with
    /// `standard == pcm` (35.9.8).
    bool pcm_long_frame = false;
    /// CKPOL: the steady level of CK. NOT a sampling-edge choice - the
    /// chapter's own note says CKPOL "does not affect the CK edge
    /// sensitivity used to receive or transmit the SD and WS signals".
    bool clock_idle_high = false;
    /// ASTRTEN (35.9.8): a slave that starts on the WS LEVEL rather
    /// than on its transition, so a receiver joining a stream already
    /// running does not wait for the next frame boundary.
    bool asynchronous_start = false;
    /// I2SPR, master only: the linear prescaler, its odd bit and the
    /// master-clock output. I2SDIV must be strictly above 1.
    uint8_t divider = 2;
    bool divider_odd = false;
    bool master_clock_out = false;
};

/// 35.9.9: I2SDIV = 0 and 1 are forbidden values, and the master clock
/// and prescaler mean nothing on a slave (they are simply not written).
constexpr bool i2s_config_valid(const I2sConfig& c) {
    if (!i2s_is_master(c.mode)) {
        return true;
    }
    return c.divider >= 2u;
}

/**
 * The sampling frequency a prescaler produces, 35.7.4's own four
 * formulas as one function: the denominator is 256 (I2S with MCK), 128
 * (PCM with MCK), or 32 / 16 times (CHLEN + 1) without it, all over
 * (2 x I2SDIV + ODD). Returns 0 for a configuration that has no answer
 * (a slave, or a forbidden divider) rather than dividing by zero.
 */
constexpr uint32_t i2s_sampling_hz(uint32_t ker_hz, const I2sConfig& c) {
    if (!i2s_is_master(c.mode) || c.divider < 2u) {
        return 0;
    }
    const uint32_t div = 2u * static_cast<uint32_t>(c.divider) + (c.divider_odd ? 1u : 0u);
    const bool pcm = c.standard == I2sStandard::pcm;
    uint32_t factor;
    if (c.master_clock_out) {
        factor = pcm ? 128u : 256u;
    } else {
        const uint32_t chlen = i2s_channel_bits(c.data_length, c.channel_length) == 32u ? 2u : 1u;
        factor = (pcm ? 16u : 32u) * chlen;
    }
    return ker_hz / (factor * div);
}

/**
 * The prescaler pair that comes CLOSEST to `fs` at this kernel clock -
 * the inverse of i2s_sampling_hz(), searched rather than solved because
 * the ODD bit makes the divisor a half-integer ladder. Nullopt when the
 * answer would need a divider below 2 or above 255.
 *
 * `base` is only used for its standard, its lengths and its MCK flag;
 * the returned copy carries the divider and the odd bit filled in.
 */
constexpr std::optional<I2sConfig> i2s_prescaler_for(uint32_t ker_hz, uint32_t fs,
                                                     I2sConfig base) {
    if (ker_hz == 0u || fs == 0u || !i2s_is_master(base.mode)) {
        return {};
    }
    const bool pcm = base.standard == I2sStandard::pcm;
    uint32_t factor;
    if (base.master_clock_out) {
        factor = pcm ? 128u : 256u;
    } else {
        const uint32_t chlen = i2s_channel_bits(base.data_length, base.channel_length) == 32u ? 2u : 1u;
        factor = (pcm ? 16u : 32u) * chlen;
    }
    // The exact real divisor, rounded to nearest half-integer: div2 is
    // (2 x I2SDIV + ODD), so search that integer directly.
    const uint32_t want = (ker_hz + (factor * fs) / 2u) / (factor * fs);
    if (want < 4u || want > 511u) {
        return {};
    }
    base.divider = static_cast<uint8_t>(want / 2u);
    base.divider_odd = (want & 1u) != 0u;
    if (base.divider < 2u) {
        return {};
    }
    return base;
}

constexpr uint32_t i2s_cfgr(const I2sConfig& c) {
    uint32_t v = SPI_I2SCFGR_I2SMOD;
    if (c.asynchronous_start) {
        v |= SPI_I2SCFGR_ASTRTEN;
    }
    v |= static_cast<uint32_t>(c.mode) << SPI_I2SCFGR_I2SCFG_Pos;
    if (c.standard == I2sStandard::pcm && c.pcm_long_frame) {
        v |= SPI_I2SCFGR_PCMSYNC;
    }
    v |= static_cast<uint32_t>(c.standard) << SPI_I2SCFGR_I2SSTD_Pos;
    if (c.clock_idle_high) {
        v |= SPI_I2SCFGR_CKPOL;
    }
    v |= static_cast<uint32_t>(c.data_length) << SPI_I2SCFGR_DATLEN_Pos;
    if (c.channel_length == I2sChannelLength::bits32) {
        v |= SPI_I2SCFGR_CHLEN;
    }
    return v;
}

constexpr uint32_t i2s_pr(const I2sConfig& c) {
    uint32_t v = c.divider;
    if (c.divider_odd) {
        v |= SPI_I2SPR_ODD;
    }
    if (c.master_clock_out) {
        v |= SPI_I2SPR_MCKOE;
    }
    return v;
}

/// RCC_CCIPR(2)'s I2SnSEL codes, the same four in both registers
/// (5.4.21, 5.4.22).
enum class I2sClock : uint8_t { sysclk = 0, pllpclk = 1, hsi16 = 2, ckin = 3 };

// =============================================================================
// The resource
// =============================================================================

template <uint8_t n>
struct Spi {
    static_assert(spi_present(n),
                  "brio Spi: this device has no such SPI instance (the device header "
                  "declares no SPIn_BASE for it: SPI1 and SPI2 on every STM32G0, SPI3 "
                  "only on the G0B1/G0C1 class)");

    Spi() = delete;

    static constexpr uint8_t index = n;

    /// Table 205's "I2S support" row, from the reserve (which derives it
    /// from the presence of a SPI3, the footnote's own condition).
    static constexpr bool has_i2s_mode = spi_has_i2s(n);
    /// Every instance of this family has the enhanced NSSP and TI modes,
    /// hardware CRC, 4..16-bit frames and 32-bit FIFOs - table 205's
    /// other rows are the same in all three columns. Stated as constants
    /// so the bench can compare a table with a register.
    static constexpr bool has_nss_pulse = true;
    static constexpr bool has_ti_mode = true;
    static constexpr bool has_crc = true;
    static constexpr uint8_t fifo_bits = 32;
    /// Table 205's last row: "Wake-up capability from Low-power Sleep".
    static constexpr bool wakes_from_low_power_sleep = true;

    static SPI_TypeDef& regs() { return *reinterpret_cast<SPI_TypeDef*>(spi_base(n)); }
    static constexpr IRQn_Type irq() { return spi_irq(n); }

    /// The DMAMUX request ids of this instance (table 56), published
    /// BY THE PERIPHERAL - stm32g0/dma.hpp takes a plain number and
    /// knows nothing about SPIs (the EVSYS ruling, kept).
    static constexpr uint8_t dma_rx_request() { return spi_dma_rx_request(n); }
    static constexpr uint8_t dma_tx_request() { return spi_dma_tx_request(n); }
    /// The address an engine pours into or drains from. It is ONE
    /// register for both directions and both widths; which width an
    /// access uses is the engine's element type.
    static volatile void* data_address() { return &regs().DR; }

    /**
     * THE HEADER'S OWN ANSWER to the I2S question, at run time.
     * IS_I2S_ALL_INSTANCE(x) is a pointer comparison - a fine expression
     * and a useless constant one - so it cannot key the reserve, but
     * folded against a constant &regs() it costs nothing and is the
     * SECOND opinion the bench compares the reserve's table and the
     * silicon against (test_stm32_spi letter a). The usart_is_full()
     * precedent, applied.
     */
    static bool has_i2s() { return IS_I2S_ALL_INSTANCE(&regs()) != 0; }

    // ---- clocks and reset ---------------------------------------------------

    /// The APB clock of the block. Dead registers without it.
    static void bus_clock(bool on) {
        constexpr SpiBusClock bc = spi_bus_clock(n);
        if constexpr (bc.apb2) {
            Rcc::apb2_clock(bc.mask, on);
        } else {
            Rcc::apb1_clock(bc.mask, on);
        }
    }
    static bool bus_clock() {
        constexpr SpiBusClock bc = spi_bus_clock(n);
        if constexpr (bc.apb2) {
            return Rcc::apb2_clock(bc.mask);
        } else {
            return Rcc::apb1_clock(bc.mask);
        }
    }

    /// Software reset through RCC: every register back to its reset
    /// value, which is the one way out of a state the chapter has no
    /// sequence for (35.5.9 names it itself: "by initializing all the
    /// SPI registers with a software reset via ... the SPIiRST bits").
    static void reset() {
        constexpr SpiBusClock bc = spi_bus_clock(n);
        if constexpr (bc.apb2) {
            Rcc::apb2_reset(bc.mask);
        } else {
            Rcc::apb1_reset(bc.mask);
        }
    }

    /// I2SnSEL, where the instance has an I2S. False - with NOTHING
    /// written - on an instance that has not, and on a part whose
    /// selector register this device does not carry.
    static bool i2s_kernel_clock(I2sClock c) {
        constexpr I2sClockSelect sel = i2s_clock_select(n);
        if constexpr (sel.pos == 0xFF) {
            (void)c;
            return false;
        } else if constexpr (sel.ccipr2) {
            return Rcc::kernel_clock2(sel.pos, static_cast<uint8_t>(c));
        } else {
            Rcc::kernel_clock(sel.pos, static_cast<uint8_t>(c));
            return true;
        }
    }
    static I2sClock i2s_kernel_clock() {
        constexpr I2sClockSelect sel = i2s_clock_select(n);
        if constexpr (sel.pos == 0xFF) {
            return I2sClock::sysclk;
        } else if constexpr (sel.ccipr2) {
            return static_cast<I2sClock>(Rcc::kernel_clock2(sel.pos) & 0x3u);
        } else {
            return static_cast<I2sClock>(Rcc::kernel_clock(sel.pos));
        }
    }

    // ---- the enable, and the disable PROCEDURE ------------------------------

    static bool enabled() { return (regs().CR1 & SPI_CR1_SPE) != 0u; }

    /// Turn the peripheral on. 35.5.8: a CLIENT should be enabled - and
    /// hold its first datum - before the master's clock arrives, which
    /// is why this is a verb of its own and not the tail of configure().
    static void enable() { regs().CR1 = regs().CR1 | SPI_CR1_SPE; }

    /**
     * 35.5.9's DISABLE PROCEDURE, and the only way this driver turns the
     * peripheral off:
     *   1. wait FTLVL = 00 (nothing left to transmit)
     *   2. wait BSY = 0    (the last frame is through)
     *   3. clear SPE
     *   4. read DR until FRLVL = 00 (nothing left unread)
     *
     * ES0548 2.12.1's workaround falls out of steps 1 and 2 for the
     * master-transmit case it names. Both waits are BOUNDED - the
     * slowest frame this generator can produce is 256 x 16 PCLK cycles -
     * and a wait that runs out returns false with SPE cleared anyway:
     * an engine that reports is worth more than one that hangs.
     *
     * @return false when a wait ran out (the flags are left readable for
     * whoever wants to say WHICH).
     */
    static bool disable() {
        constexpr uint32_t spins = 200'000u;
        bool ok = true;
        uint32_t left = spins;
        while ((regs().SR & SPI_SR_FTLVL) != 0u && left-- != 0u) {
        }
        ok = ok && left != 0u;
        left = spins;
        while ((regs().SR & SPI_SR_BSY) != 0u && left-- != 0u) {
        }
        ok = ok && left != 0u;
        regs().CR1 = regs().CR1 & ~SPI_CR1_SPE;
        flush_rx();
        return ok;
    }

    /**
     * 35.5.9's OTHER disable procedure, for the receive-only modes whose
     * clock only stops when SPE does: clear SPE first, then wait BSY,
     * then drain. The caller owns the timing - the chapter's "within the
     * specific time window ... between the sampling time of its first
     * bit and before its last bit transfer starts" is a decision only a
     * program that knows the frame boundaries can make.
     *
     * ES0548 2.12.1's second case says BSY is unreliable HERE, so the
     * wait is bounded and its result is reported rather than trusted.
     */
    static bool disable_receive_only() {
        regs().CR1 = regs().CR1 & ~SPI_CR1_SPE;
        uint32_t left = 200'000u;
        while ((regs().SR & SPI_SR_BSY) != 0u && left-- != 0u) {
        }
        const bool ok = left != 0u;
        flush_rx();
        return ok;
    }

    /// Read DR until FRLVL says the RXFIFO is empty (step 4 on its own,
    /// for the paths that need it without the whole procedure - a fresh
    /// transaction must not capture the previous one's leftovers).
    static void flush_rx() {
        uint32_t left = 8u;   // the FIFO is four frames deep at most
        while ((regs().SR & SPI_SR_FRLVL) != 0u && left-- != 0u) {
            (void)*dr_byte_view();
        }
    }

    // ---- configuration: every one of these refuses while SPE is set ---------

    /// The whole of CR1 and CR2 in one pair of stores, plus CRCPR.
    /// Refused - nothing written - while the instance is enabled or the
    /// configuration breaks one of the chapter's own rules.
    static bool configure(const SpiConfig& c) {
        if (enabled() || !spi_config_valid(c)) {
            return false;
        }
        regs().CR1 = spi_cr1(c);
        regs().CR2 = spi_cr2(c);
        regs().CRCPR = c.crc_polynomial;
        return true;
    }

    static bool role(SpiRole r) {
        if (enabled()) {
            return false;
        }
        regs().CR1 = r == SpiRole::host ? (regs().CR1 | SPI_CR1_MSTR)
                                        : (regs().CR1 & ~SPI_CR1_MSTR);
        return true;
    }
    static SpiRole role() {
        return (regs().CR1 & SPI_CR1_MSTR) != 0u ? SpiRole::host : SpiRole::client;
    }

    /// CPOL and CPHA together - 35.5.6's Note makes them one act ("prior
    /// to changing the CPOL/CPHA bits the SPI must be disabled").
    static bool mode(SpiMode m) {
        if (enabled()) {
            return false;
        }
        uint32_t v = regs().CR1 & ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
        if (spi_cpol(m)) {
            v |= SPI_CR1_CPOL;
        }
        if (spi_cpha(m)) {
            v |= SPI_CR1_CPHA;
        }
        regs().CR1 = v;
        return true;
    }
    static SpiMode mode() {
        const uint32_t v = regs().CR1;
        return static_cast<SpiMode>(((v & SPI_CR1_CPOL) != 0u ? 2u : 0u) |
                                    ((v & SPI_CR1_CPHA) != 0u ? 1u : 0u));
    }

    static bool clock(SpiClock c) {
        if (enabled()) {
            return false;
        }
        regs().CR1 = (regs().CR1 & ~SPI_CR1_BR) |
                     (static_cast<uint32_t>(c) << SPI_CR1_BR_Pos);
        return true;
    }
    static SpiClock clock() {
        return static_cast<SpiClock>((regs().CR1 & SPI_CR1_BR) >> SPI_CR1_BR_Pos);
    }

    static bool bit_order(bool lsb_first) {
        if (enabled()) {
            return false;
        }
        regs().CR1 = lsb_first ? (regs().CR1 | SPI_CR1_LSBFIRST)
                               : (regs().CR1 & ~SPI_CR1_LSBFIRST);
        return true;
    }
    static bool lsb_first() { return (regs().CR1 & SPI_CR1_LSBFIRST) != 0u; }

    /// DS and FRXTH move together: the threshold IS the access size
    /// (35.5.7 step 3e), so a data size written on its own would leave
    /// the receiver reading at the wrong granularity. Pass a threshold
    /// only for packed traffic.
    static bool data_size(SpiDataSize d, std::optional<SpiRxThreshold> th = {}) {
        if (enabled() || !spi_data_size_valid(d)) {
            return false;
        }
        const SpiRxThreshold t =
            th.value_or(spi_frame_is_halfword(d) ? SpiRxThreshold::half
                                                 : SpiRxThreshold::quarter);
        uint32_t v = (regs().CR2 & ~(SPI_CR2_DS | SPI_CR2_FRXTH)) |
                     (static_cast<uint32_t>(d) << SPI_CR2_DS_Pos);
        if (t == SpiRxThreshold::quarter) {
            v |= SPI_CR2_FRXTH;
        }
        regs().CR2 = v;
        return true;
    }
    static SpiDataSize data_size() {
        return static_cast<SpiDataSize>((regs().CR2 & SPI_CR2_DS) >> SPI_CR2_DS_Pos);
    }
    /// FRXTH on its own - the ONE verb of this group that is legal under
    /// a running SPI, and deliberately so: 35.5.9's own note makes the
    /// last odd frame of a packed reception readable by MOVING the
    /// threshold when FRLVL = 01, which can only happen mid-transfer.
    static void rx_threshold(SpiRxThreshold t) {
        regs().CR2 = t == SpiRxThreshold::quarter ? (regs().CR2 | SPI_CR2_FRXTH)
                                                  : (regs().CR2 & ~SPI_CR2_FRXTH);
    }
    static SpiRxThreshold rx_threshold() {
        return (regs().CR2 & SPI_CR2_FRXTH) != 0u ? SpiRxThreshold::quarter
                                                  : SpiRxThreshold::half;
    }

    static bool direction(SpiDirection d) {
        if (enabled()) {
            return false;
        }
        uint32_t v = regs().CR1 & ~(SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_RXONLY);
        if (spi_direction_is_bidi(d)) {
            v |= SPI_CR1_BIDIMODE;
            if (d == SpiDirection::bidi_transmit) {
                v |= SPI_CR1_BIDIOE;
            }
        } else if (d == SpiDirection::receive_only) {
            v |= SPI_CR1_RXONLY;
        }
        regs().CR1 = v;
        return true;
    }
    static SpiDirection direction() {
        const uint32_t v = regs().CR1;
        if ((v & SPI_CR1_BIDIMODE) != 0u) {
            return (v & SPI_CR1_BIDIOE) != 0u ? SpiDirection::bidi_transmit
                                              : SpiDirection::bidi_receive;
        }
        return (v & SPI_CR1_RXONLY) != 0u ? SpiDirection::receive_only
                                          : SpiDirection::full_duplex;
    }
    /// The ONE bidirectional knob that is meant to move under a running
    /// SPI: BIDIOE turns the single data line round (35.9.1 puts no
    /// enable condition on it, unlike BIDIMODE's neighbours).
    static void bidi_output(bool on) {
        regs().CR1 = on ? (regs().CR1 | SPI_CR1_BIDIOE) : (regs().CR1 & ~SPI_CR1_BIDIOE);
    }

    static bool nss(SpiNss arrangement) {
        if (enabled()) {
            return false;
        }
        uint32_t c1 = regs().CR1 & ~SPI_CR1_SSM;
        uint32_t c2 = regs().CR2 & ~(SPI_CR2_SSOE | SPI_CR2_NSSP);
        if (arrangement == SpiNss::software) {
            c1 |= SPI_CR1_SSM;
        }
        if (spi_nss_is_output(arrangement)) {
            c2 |= SPI_CR2_SSOE;
        }
        if (arrangement == SpiNss::hardware_pulse) {
            c2 |= SPI_CR2_NSSP;
        }
        regs().CR1 = c1;
        regs().CR2 = c2;
        return true;
    }
    static SpiNss nss() {
        if ((regs().CR1 & SPI_CR1_SSM) != 0u) {
            return SpiNss::software;
        }
        if ((regs().CR2 & SPI_CR2_NSSP) != 0u) {
            return SpiNss::hardware_pulse;
        }
        return (regs().CR2 & SPI_CR2_SSOE) != 0u ? SpiNss::hardware_output
                                                 : SpiNss::hardware_input;
    }

    /// CR1.SSI - the internal select a software-NSS instance presents to
    /// itself. LEGAL UNDER A RUNNING SPI on purpose: pulling it low
    /// under a live master is how a program stages a mode fault, and
    /// holding it low is how a software-NSS client selects itself.
    static void software_select(bool selected) {
        regs().CR1 = selected ? (regs().CR1 & ~SPI_CR1_SSI)
                              : (regs().CR1 | SPI_CR1_SSI);
    }
    /// True while the internal select says "selected" (SSI LOW).
    static bool software_selected() { return (regs().CR1 & SPI_CR1_SSI) == 0u; }

    static bool frame_format(SpiFrameFormat f) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = f == SpiFrameFormat::ti ? (regs().CR2 | SPI_CR2_FRF)
                                             : (regs().CR2 & ~SPI_CR2_FRF);
        return true;
    }
    static SpiFrameFormat frame_format() {
        return (regs().CR2 & SPI_CR2_FRF) != 0u ? SpiFrameFormat::ti
                                                : SpiFrameFormat::motorola;
    }

    // ---- CRC (35.5.14) ------------------------------------------------------

    static bool crc(bool on, SpiCrcLength len = SpiCrcLength::crc8) {
        if (enabled()) {
            return false;
        }
        uint32_t v = regs().CR1 & ~(SPI_CR1_CRCEN | SPI_CR1_CRCL);
        if (on) {
            v |= SPI_CR1_CRCEN;
            if (len == SpiCrcLength::crc16) {
                v |= SPI_CR1_CRCL;
            }
        }
        regs().CR1 = v;
        return true;
    }
    static bool crc() { return (regs().CR1 & SPI_CR1_CRCEN) != 0u; }
    static SpiCrcLength crc_length() {
        return (regs().CR1 & SPI_CR1_CRCL) != 0u ? SpiCrcLength::crc16 : SpiCrcLength::crc8;
    }

    /// 35.9.5: the polynomial must be ODD, and the silicon does not
    /// check - so this verb does, and refuses an even one before a
    /// register is touched.
    static bool crc_polynomial(uint16_t poly) {
        if (enabled() || (poly & 1u) == 0u) {
            return false;
        }
        regs().CRCPR = poly;
        return true;
    }
    static uint16_t crc_polynomial() { return static_cast<uint16_t>(regs().CRCPR); }

    /**
     * CR1.CRCNEXT: the next frame out is the transmit checksum. LEGAL
     * UNDER A RUNNING SPI by necessity - 35.9.1 says it "has to be
     * written as soon as the last data is written in the SPIx_DR
     * register", which is the middle of a transfer by definition.
     */
    static void crc_next() { regs().CR1 = regs().CR1 | SPI_CR1_CRCNEXT; }
    static bool crc_next_pending() { return (regs().CR1 & SPI_CR1_CRCNEXT) != 0u; }

    static uint16_t rx_crc() { return static_cast<uint16_t>(regs().RXCRCR); }
    static uint16_t tx_crc() { return static_cast<uint16_t>(regs().TXCRCR); }

    /**
     * 35.5.14's "if the SPI is disabled during a communication" reset of
     * the two checksum accumulators: disable, clear CRCEN, set it again,
     * enable. The caller gets the peripheral back exactly as it handed
     * it over - which is why this is one verb and not four.
     * @return the disable procedure's own answer.
     */
    static bool restart_crc() {
        const bool was_enabled = enabled();
        const bool ok = was_enabled ? disable() : true;
        regs().CR1 = regs().CR1 & ~SPI_CR1_CRCEN;
        regs().CR1 = regs().CR1 | SPI_CR1_CRCEN;
        if (was_enabled) {
            enable();
        }
        return ok;
    }

    // ---- DMA (35.5.9) -------------------------------------------------------

    static bool dma_transmit(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = on ? (regs().CR2 | SPI_CR2_TXDMAEN) : (regs().CR2 & ~SPI_CR2_TXDMAEN);
        return true;
    }
    static bool dma_receive(bool on) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = on ? (regs().CR2 | SPI_CR2_RXDMAEN) : (regs().CR2 & ~SPI_CR2_RXDMAEN);
        return true;
    }
    static bool dma_transmit() { return (regs().CR2 & SPI_CR2_TXDMAEN) != 0u; }
    static bool dma_receive() { return (regs().CR2 & SPI_CR2_RXDMAEN) != 0u; }

    /// LDMA_TX / LDMA_RX (35.9.2): "the total number of data to transmit
    /// by DMA is odd", which the silicon needs to know because a packed
    /// 16-bit access carries TWO frames and the last one would otherwise
    /// drag a dummy along. Enable-protected by the register's own words.
    static bool last_dma_transmit_odd(bool odd) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = odd ? (regs().CR2 | SPI_CR2_LDMATX) : (regs().CR2 & ~SPI_CR2_LDMATX);
        return true;
    }
    static bool last_dma_receive_odd(bool odd) {
        if (enabled()) {
            return false;
        }
        regs().CR2 = odd ? (regs().CR2 | SPI_CR2_LDMARX) : (regs().CR2 & ~SPI_CR2_LDMARX);
        return true;
    }

    // ---- the data register, in both widths ----------------------------------

    /**
     * THE TWO ACCESS WIDTHS ARE TWO DIFFERENT REGISTERS as far as this
     * FIFO is concerned, and the device header declares only the wide
     * one (`__IO uint32_t DR`). So the narrow views are made from the
     * SAME address, through an integer: a pointer-to-pointer cast is
     * what -Wstrict-aliasing objects to, and going by way of uintptr_t
     * says what is meant - "the byte at this address" - without asking
     * the compiler to believe two incompatible pointers alias.
     */
    static volatile uint8_t* dr_byte_view() {
        return reinterpret_cast<volatile uint8_t*>(reinterpret_cast<uintptr_t>(&regs().DR));
    }
    static volatile uint16_t* dr_halfword_view() {
        return reinterpret_cast<volatile uint16_t*>(reinterpret_cast<uintptr_t>(&regs().DR));
    }

    /// An 8-bit access: one frame with DS <= 8, or the LOW frame of a
    /// packed pair. Writing DR as a byte is the ONLY way to put a single
    /// short frame in the FIFO (35.5.9's "Data packing").
    static void data_byte(uint8_t v) { *dr_byte_view() = v; }
    static uint8_t data_byte() { return *dr_byte_view(); }

    /// A 16-bit access: one frame with DS > 8, or a PACKED PAIR of short
    /// ones. The FIFO does not care which - the frame size does.
    static void data_halfword(uint16_t v) { *dr_halfword_view() = v; }
    static uint16_t data_halfword() { return *dr_halfword_view(); }

    /// One frame of the stated size, the access width picked for the
    /// caller (the rule of figure 363, as code).
    static void data(SpiDataSize d, uint16_t v) {
        if (spi_frame_is_halfword(d)) {
            data_halfword(v);
        } else {
            data_byte(static_cast<uint8_t>(v));
        }
    }
    static uint16_t data(SpiDataSize d) {
        return spi_frame_is_halfword(d) ? data_halfword() : data_byte();
    }

    // ---- status -------------------------------------------------------------

    static uint32_t status() { return regs().SR; }
    static bool rxne() { return (regs().SR & SPI_SR_RXNE) != 0u; }
    static bool txe() { return (regs().SR & SPI_SR_TXE) != 0u; }
    static bool busy() { return (regs().SR & SPI_SR_BSY) != 0u; }
    static bool channel_right() { return (regs().SR & SPI_SR_CHSIDE) != 0u; }

    /// FTLVL / FRLVL as frame-ish quarters, 0..3 (35.9.3). They are the
    /// only honest witness of what is still in flight, BSY being what
    /// ES0548 2.12.2 says it is.
    static uint8_t tx_level() {
        return static_cast<uint8_t>((regs().SR & SPI_SR_FTLVL) >> SPI_SR_FTLVL_Pos);
    }
    static uint8_t rx_level() {
        return static_cast<uint8_t>((regs().SR & SPI_SR_FRLVL) >> SPI_SR_FRLVL_Pos);
    }

    static bool overrun() { return (regs().SR & SPI_SR_OVR) != 0u; }
    static bool mode_fault() { return (regs().SR & SPI_SR_MODF) != 0u; }
    static bool crc_error() { return (regs().SR & SPI_SR_CRCERR) != 0u; }
    static bool frame_error() { return (regs().SR & SPI_SR_FRE) != 0u; }
    static bool underrun() { return (regs().SR & SPI_SR_UDR) != 0u; }

    /// 35.5.11's OVR sequence: "a read access to the SPI_DR register
    /// followed by a read access to the SPI_SR register". Not a W1C, not
    /// a store - two reads in that order, which is why it is a verb.
    static void clear_overrun() {
        (void)regs().DR;
        (void)regs().SR;
    }

    /**
     * 35.5.11's MODF sequence: "a read or write access to the SPIx_SR
     * register while the MODF bit is set. Then write to the SPIx_CR1
     * register." The silicon has already cleared SPE and MSTR for the
     * program - it forced this instance into slave mode - so the caller
     * decides what to restore; this verb only performs the ritual and
     * hands CR1 back unchanged.
     */
    static void clear_mode_fault() {
        (void)regs().SR;
        regs().CR1 = regs().CR1;
    }

    /// 35.9.3: CRCERR is rc_w0 - "cleared by software writing 0".
    static void clear_crc_error() { regs().SR = regs().SR & ~SPI_SR_CRCERR; }

    /// 35.5.11: "The FRE flag is cleared when SPIx_SR register is read."
    static void clear_frame_error() { (void)regs().SR; }

    /// The I2S underrun (35.7.8), cleared the way OVR is: read SR.
    static void clear_underrun() { (void)regs().SR; }

    // ---- interrupts ---------------------------------------------------------

    static void interrupts(uint32_t mask, bool on) {
        InterruptGuard guard;   // CR2 is read-modify-write and shared with the handler
        regs().CR2 = on ? (regs().CR2 | mask) : (regs().CR2 & ~mask);
    }
    static uint32_t interrupts() { return regs().CR2 & SpiInterrupt::all; }
    static void rxne_interrupt(bool on) { interrupts(SpiInterrupt::rxne, on); }
    static void txe_interrupt(bool on) { interrupts(SpiInterrupt::txe, on); }
    static void error_interrupt(bool on) { interrupts(SpiInterrupt::error, on); }

    /**
     * The instance's ISR BODY: which sources are both raised AND
     * enabled, as one mask. It decides nothing and clears nothing - what
     * a received frame means is the caller's, and reading DR is the
     * caller's too, because only the caller knows where the frame goes.
     *
     * ERRIE is one enable for six flags, so the error bits come back
     * together and the caller's handler is what tells OVR from MODF.
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t sr = regs().SR;
        const uint32_t cr2 = regs().CR2;
        uint32_t served = 0;
        if ((cr2 & SPI_CR2_RXNEIE) != 0u) {
            served |= sr & SpiFlag::rxne;
        }
        if ((cr2 & SPI_CR2_TXEIE) != 0u) {
            served |= sr & SpiFlag::txe;
        }
        if ((cr2 & SPI_CR2_ERRIE) != 0u) {
            served |= sr & SpiFlag::errors;
        }
        return served;
    }

    // ---- the I2S personality ------------------------------------------------

    static bool i2s_mode() { return (regs().I2SCFGR & SPI_I2SCFGR_I2SMOD) != 0u; }
    static bool i2s_enabled() { return (regs().I2SCFGR & SPI_I2SCFGR_I2SE) != 0u; }

    /**
     * Put the block in I2S mode and configure it whole (35.7.5's steps
     * 1..3 in the chapter's own order: the prescaler first, then the
     * configuration register with I2SMOD in it). Refused - nothing
     * written - on an instance table 205 gives no I2S, while the SPI is
     * enabled, while the I2S already is, or on a configuration that
     * breaks 35.9.9's divider rule.
     */
    static bool i2s_configure(const I2sConfig& c) {
        if constexpr (!has_i2s_mode) {
            (void)c;
            return false;
        } else {
            if (enabled() || i2s_enabled() || !i2s_config_valid(c)) {
                return false;
            }
            regs().I2SPR = i2s_pr(c);
            regs().I2SCFGR = i2s_cfgr(c);
            return true;
        }
    }

    /// Back to SPI Motorola mode: I2SMOD cleared, with the I2S off.
    /// Refused while either enable stands.
    static bool spi_mode() {
        if (enabled() || i2s_enabled()) {
            return false;
        }
        regs().I2SCFGR = 0;
        return true;
    }

    /// 35.7.5 step 5. Nothing else may be written after it.
    static bool i2s_enable() {
        if constexpr (!has_i2s_mode) {
            return false;
        } else {
            if (!i2s_mode()) {
                return false;
            }
            regs().I2SCFGR = regs().I2SCFGR | SPI_I2SCFGR_I2SE;
            return true;
        }
    }

    /**
     * 35.7.5's transmitter shutdown: "to switch off the I2S, by clearing
     * I2SE, it is mandatory to wait for TXE = 1 and BSY = 0".
     * @return false when either wait ran out (bounded, as everywhere).
     */
    static bool i2s_stop_transmit() {
        constexpr uint32_t spins = 400'000u;
        uint32_t left = spins;
        while ((regs().SR & SPI_SR_TXE) == 0u && left-- != 0u) {
        }
        bool ok = left != 0u;
        left = spins;
        while ((regs().SR & SPI_SR_BSY) != 0u && left-- != 0u) {
        }
        ok = ok && left != 0u;
        regs().I2SCFGR = regs().I2SCFGR & ~SPI_I2SCFGR_I2SE;
        return ok;
    }

    /**
     * The RECEIVER's shutdown is I2SE cleared and nothing else, and the
     * WAIT BEFORE IT IS THE CALLER'S: 35.7.5 gives three different
     * sequences keyed by DATLEN, CHLEN and I2SSTD, each of them "wait
     * for the (second to) last RXNE, then wait N I2S clock cycles using
     * a software loop" - and how long an I2S clock cycle is depends on a
     * prescaler this side of a slave link does not own. A driver that
     * guessed would be stating a fact it cannot enforce.
     */
    static void i2s_disable() { regs().I2SCFGR = regs().I2SCFGR & ~SPI_I2SCFGR_I2SE; }

    static I2sMode i2s_configuration() {
        return static_cast<I2sMode>((regs().I2SCFGR & SPI_I2SCFGR_I2SCFG) >>
                                    SPI_I2SCFGR_I2SCFG_Pos);
    }
    static I2sStandard i2s_standard() {
        return static_cast<I2sStandard>((regs().I2SCFGR & SPI_I2SCFGR_I2SSTD) >>
                                        SPI_I2SCFGR_I2SSTD_Pos);
    }
    static I2sDataLength i2s_data_length() {
        return static_cast<I2sDataLength>((regs().I2SCFGR & SPI_I2SCFGR_DATLEN) >>
                                          SPI_I2SCFGR_DATLEN_Pos);
    }
    static I2sChannelLength i2s_channel_length() {
        return (regs().I2SCFGR & SPI_I2SCFGR_CHLEN) != 0u ? I2sChannelLength::bits32
                                                          : I2sChannelLength::bits16;
    }
    static uint8_t i2s_divider() { return static_cast<uint8_t>(regs().I2SPR & 0xFFu); }
    static bool i2s_divider_odd() { return (regs().I2SPR & SPI_I2SPR_ODD) != 0u; }
    static bool i2s_master_clock_out() { return (regs().I2SPR & SPI_I2SPR_MCKOE) != 0u; }
};

// =============================================================================
// The engine slot tag
// =============================================================================

/*
 * `NoDmaEngine` comes from stm32g0/dma_engine.hpp, and it lives in a
 * file of its own for the reason that file states: a driver with an
 * optional engine slot must not include stm32g0/dma.hpp, or every
 * program with a bus would carry the DMA controller. This file reaches
 * its engines only through their own published names - start(),
 * start_fixed(), start_discard(), service(), abandon(), stop(),
 * flag_complete, flag_error - and never spells a DmaChannel.
 */

/// Both engines or neither, and never the same channel twice - the
/// UartTask rule, restated for a bus whose completion is the RECEIVE
/// block's.
template <typename Tx, typename Rx>
constexpr bool spi_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::controller != Rx::controller || Tx::channel != Rx::channel;
    } else {
        return true;
    }
}

/// A DMA transfer error as a BusDone status - the first engine-defined
/// code bus_master.hpp reserves, spelled the same way samc21/spi.hpp
/// spells it. On an engineless host status() is spi_ok by construction
/// and this code is unreachable.
inline constexpr uint8_t spi_dma_fault = bus_engine_status;

// =============================================================================
// The host task
// =============================================================================

/**
 * SpiHost<n, pins, TxEngine, RxEngine>
 *
 * The transfer ENGINE util/spi_bus.hpp drives (which owns arbitration,
 * the pending FIFO, replies and the per-bus timeout). This task owns the
 * wire: the chip select, the D/C line of display-style devices, and the
 * frame pump under the SPI interrupt.
 *
 * TRANSACTION DESCRIPTOR (Request) - two phases in ONE chip-select
 * window, the SAME shape avrdx/spi.hpp and samc21/spi.hpp carry:
 *
 *   phase 1 (optional): cmd[cmd_len] transmitted with DC LOW
 *   phase 2 (optional): len frames with DC HIGH, FULL-DUPLEX -
 *                       transmit tx[] (or 0xFF dummies if tx == null),
 *                       capture into rx[] (or discard if rx == null)
 *
 * CS is ACTIVE LOW and asserted/released by the engine around the whole
 * transaction; dc may be a null PinRef. Buffer ownership travels with
 * the request (Lease::reply: the client hands the spans off until its
 * SpiDone comes back). A zero-total-length request completes on the spot
 * without touching the wire.
 *
 * LEN IS A FRAME COUNT AND THE FRAME SIZE IS THE REQUEST'S. With the
 * default 8-bit frames a frame is a byte and `len` is a byte count -
 * exactly the other two targets' meaning, which is what lets a device
 * client compile unchanged. With `bits` above 8 a frame occupies TWO
 * bytes of the caller's buffer, in the CPU's own order (low byte first),
 * and `len` still counts frames: a 16-bit request of len 4 moves four
 * frames out of eight bytes. Below 8 bits the frame still occupies one
 * byte, right-aligned, with the unused top bits ignored on the way out
 * and read back as zero (figure 363).
 *
 * WHY THE CHIP SELECT IS A GPIO. 35.5.5: hardware NSS output "is driven
 * low as soon as the SPI is enabled in master mode (SPE = 1), and is
 * kept low until the SPI is disabled" - it frames the peripheral's
 * lifetime, not a transaction - and NSS pulse mode (35.5.12) raises NSS
 * between two consecutive DATA FRAMES. Every device this engine is for
 * wants a select that spans a command and its answer, so the Request
 * carries a PinRef exactly as on the AVR and the SAM, and the two
 * hardware arrangements stay resource facts the bench measures.
 *
 * WHY RXNE AND NOT TXE DRIVES THE PUMP: the file comment says it. One
 * interrupt per frame, and the transfer ends when the last frame has
 * been shifted BACK IN, which is the only edge that means "the wire is
 * idle". It does mean the receiver runs even for a write-only transfer,
 * which costs the MISO pad and nothing else.
 *
 * THE PER-REQUEST CHIP-SELECT DELAY IS THE OTHER TARGETS', VERBATIM:
 * cs_setup_us is spent spinning in start() (main context, bounded by the
 * byte) between the CS assertion and the first clock, timed by
 * armv6m/delay.hpp on a rate init()/rebase() keep current. It is served
 * only while a Ticker runs (delay_us's own contract) and is capped below
 * one kernel tick - both stated there, neither reachable with a uint8_t
 * of microseconds on a 1000 Hz ticker.
 *
 * THE TWO OPTIONAL DMA ENGINE SLOTS (the Uart's shape, for the same
 * reason: an engineless build must stay byte-identical, so the slots
 * default to NoDmaEngine and every DMA branch folds away under
 * `if constexpr`). With engines named, the DATA PHASE of every request
 * runs on the DMA: the RX channel drains DR on the RXNE request, the TX
 * channel feeds it on TXE, and the transaction is complete when the
 * RECEIVE block completes - the last frame is on the wire until it has
 * been shifted back in, the same reason the pump arms RXNE. The command
 * phase stays on the frame pump (it is a few frames with a DC flip at
 * its end); a null tx feeds 0xFF from a held source, a null rx drains
 * into a held sink.
 *
 * THE ENGINES CARRY BYTES, so the DMA path serves frame sizes of eight
 * bits and below. A request with `bits` above 8 on an engined host runs
 * on the frame pump instead - stated here, measured by the suite - and
 * the alternative (a second pair of half-word engine slots) was declined
 * because it would double the task's template surface for a case the
 * pump serves correctly. The PACKED arrangement (8-bit frames through
 * 16-bit DMA accesses, with LDMA_TX/LDMA_RX for an odd count) is a
 * RESOURCE fact and not this task's: it needs the caller's own buffer
 * layout, and the bench drives it with raw engines.
 *
 * THE DMA CONTROLLER IS THE APP'S: Dma1::init() (or whatever the app
 * uses) before any engined init(). The engines arm CHANNELS of a
 * controller somebody else owns, and a driver that reset the shared
 * block would stop every other channel in the program.
 *
 * THERE IS NO KICK HERE and the absence is stm32g0/dma.hpp's own
 * measured fact: 10.4.3's handshake is level-driven, so a channel
 * enabled while TXE already stands moves its first beat by itself. The
 * SAM's rise-latched trigger - and the kick that inverts in ITS SPI mode
 * - has no counterpart on this controller.
 *
 * A DMA TRANSFER ERROR is the one failure this otherwise ACK-less bus
 * can detect: the request completes with spi_dma_fault in status()
 * instead of pretending, and CS is raised either way so the bus is
 * released.
 *
 * WHAT A CLOCK SWITCH DOES TO A TRANSFER IN FLIGHT: it breaks it. BR is
 * a DIVISION of PCLK, so a rate change moves SCK under the frame being
 * shifted and the client's sampling point with it; rebase() therefore
 * only re-resolves the ceiling and the cs_setup timing, and A DYNAMIC
 * CLOCK MUST NOT BE SWITCHED WITH A TRANSACTION IN FLIGHT. The bus AO
 * is the natural place to enforce that (it knows when the queue is
 * empty) and util/bus_master.hpp's PrepareSleep voter is the pattern; no
 * engine can enforce it from below, so it is stated and not pretended.
 *
 * ISR wiring (app glue, as usual):
 *   extern "C" void SPI1_IRQHandler() {
 *       if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{SpiHw::status()}); }
 *   }
 *   // engine users bind the channels' vectors as well:
 *   extern "C" void DMA1_Channel2_3_IRQHandler() {
 *       if (SpiHw::dma_isr()) { brio::post<SpiBus>(brio::TransferDone{SpiHw::status()}); }
 *   }
 */
template <uint8_t n, SpiPins pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class SpiHost {
    using S = Spi<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from stm32g0/dma.hpp, or NoDmaEngine (the default)");
    // BOTH OR NEITHER: the transaction's completion is the RECEIVE
    // block's, so a TX engine alone has no edge to complete on and an RX
    // engine alone would race the frame pump for DR.
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is "
                  "full-duplex and its completion is the RECEIVE block's");
    static_assert(spi_engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the two engines must ride two different DMA channels");

    static_assert(spi_pins_valid(pins),
                  "brio SpiHost: these SPI pads are not a link - SCK must be a real pad "
                  "of this device and no two signals may name the same pin");
    static_assert(pins.sck.valid() && pins.mosi.valid(),
                  "brio SpiHost: a host needs SCK and MOSI; MISO is optional only for a "
                  "write-only bus, and this engine's pump reads it (see the class "
                  "comment on why RXNE and not TXE drives it)");

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
        /// what a device's datasheet calls CS setup (the avrdx and
        /// samc21 Requests' own field, byte for byte). Spent spinning in
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

        /// Per-transaction bus configuration. On a SHARED bus every
        /// device names its own speed, mode and frame size in the
        /// request and the engine reprograms the peripheral at each
        /// start() - which costs a disable/enable pair only when
        /// something CHANGED, and nothing per frame.
        SpiClock clock = SpiClock::div16;
        SpiMode mode = SpiMode::mode0;
        /// 4..16 bits. Eight is the default, so an 8-bit request looks
        /// exactly like the other two targets'.
        SpiDataSize bits = SpiDataSize::bits8;

        /// Completion style, the client's call: false = per-frame ISR
        /// pump (the kernel keeps running between frames); true = POLLED
        /// inside start(), completing synchronously. At fast SCK the
        /// polled loop wins on every axis - a frame at PCLK/2 is sixteen
        /// CPU cycles while an interrupt entry alone costs more. The
        /// price is that THIS dispatch blocks for the whole transfer
        /// (bounded, chosen here); global interrupts stay enabled
        /// throughout, only this SPI's own RXNE is silenced.
        bool polled = false;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------

    /**
     * @brief Bring the instance up as a host: bus clock, reset,
     * configuration, pads, the NVIC line.
     *
     * Call AFTER the main clock is set up and before interrupts are
     * enabled globally; `clock` is the app's brio::Clock tag, so the
     * baud arithmetic comes from its pclk_hz and never from a second
     * statement of the rate.
     *
     * `max_sck_hz` is an optional CEILING for the whole bus: with it set
     * the engine slows any request that would exceed it and re-resolves
     * the limit after a clock change, which is what makes rebase()
     * meaningful. 0 = no ceiling.
     *
     * @return false when the ceiling cannot be produced at this clock,
     * or when the boot configuration is refused.
     */
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its SCK ceiling and its cs_setup timing "
                      "would go stale on a clock change");

        Nvic::disable(S::irq());
        ceiling_hz_ = max_sck_hz;
        // PCLK is what BR divides (35.9.1), and in this stratum PPRE is
        // pinned at 1 by every Clock task, so PCLK == HCLK == SYSCLK and
        // clock_hz() - which folds for a static clock and asks a dynamic
        // one - is the whole answer. stm32g0/clock.hpp states the
        // pinning; a family that ever unpinned it would have to hand the
        // bus prescaler in here.
        rebase(clock_hz(clock));
        if (max_sck_hz != 0 && !ceiling_) {
            return false;   // not even PCLK/256 honours it
        }

        S::bus_clock(true);
        S::reset();

        applied_ = boot_config();
        if (!S::configure(applied_)) {
            return false;
        }
        status_ = spi_ok;
        if constexpr (has_engines) {
            TxEngine::arm(S::data_address(), S::dma_tx_request());
            RxEngine::arm(S::data_address(), S::dma_rx_request());
        }

        // The pads go to the peripheral only now, with the registers
        // already holding the idle polarity: handing GPIO the pins first
        // would park an undriven pad on the bus, and a client watching
        // SCK cannot tell a glitch from a clock edge. SPE is raised LAST
        // for the same reason - with hardware NSS output it is the edge
        // that asserts NSS (35.5.5).
        SckPin::function(pins.sck.function, {.speed = PinSpeed::very_high});
        MosiPin::function(pins.mosi.function, {.speed = PinSpeed::very_high});
        if constexpr (pins.miso.valid()) {
            MisoPin::function(pins.miso.function);
        }
        // THE NSS PAD IS NOT CLAIMED. The boot configuration is
        // SpiNss::software, and 35.5.5 says of it in as many words that
        // "the external NSS pin is free for other application uses" -
        // which is exactly what this engine's chip select is: an
        // ordinary GPIO the device client owns and the Request carries.
        // The pad still travels in SpiPins so that a program which DOES
        // want the hardware arrangements (SSOE, NSSP - resource verbs)
        // states it in one place, and so that a bench can measure them.
        S::enable();
        S::flush_rx();
        Nvic::enable(S::irq());
        return true;
    }

    /**
     * @brief PCLK changed (DynamicClock fan-out).
     *
     * A Request's `clock` is a DIVISION of PCLK and scales with it by
     * itself, exactly like the AVR's SpiClock - so what is recomputed
     * here is the CEILING and the cs_setup timing. THE BUS MUST BE IDLE:
     * see the class comment.
     */
    static void rebase(uint32_t hz) {
        pclk_hz_ = hz;
        ceiling_ = ceiling_hz_ ? spi_rate_for(hz, ceiling_hz_) : std::optional<SpiClock>{};
        // The one division delay_rate() costs is paid here, never at
        // wait time (armv6m/delay.hpp's contract).
        cs_rate_ = delay_rate(hz);
    }

    /// The BR code that produces at most `hz` of SCK at the PCLK last
    /// seen - the chooser a device's datasheet limit is spoken to.
    /// Nullopt when even PCLK/256 is faster than that.
    static std::optional<SpiClock> clock_for(uint32_t hz) {
        return spi_rate_for(pclk_hz_, hz);
    }
    /// What a request at this code really runs at, ceiling included.
    static uint32_t sck_hz(SpiClock c) { return spi_sck_hz(pclk_hz_, clamp(c)); }
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    static uint32_t reference_hz() { return pclk_hz_; }

    /**
     * @brief Put the peripheral at this mode, rate and frame size NOW,
     * moving no data.
     *
     * FOR CALLERS THAT FRAME THE SELECT WINDOW THEMSELVES (Request.cs
     * null). start() applies a request's mode before it asserts the
     * request's own cs, so an engine-owned select window always opens
     * with SCK settled at the new idle level - but a caller driving CS
     * by hand inverts that order, and a mode change is a CPOL FLIP ON
     * THE WIRE: flipped inside an open select window it is one extra
     * edge, and a selected client counts it into the frame (the samc21
     * bench measured that as an exact one-bit slip, both directions).
     * Prime FIRST, then assert the select.
     */
    static void prime(SpiMode m, SpiClock c, SpiDataSize bits = SpiDataSize::bits8) {
        apply(m, clamp(c), bits);
    }

    /**
     * @brief The bus's bit order (CR1.LSBFIRST).
     *
     * NOT a per-request field, and the asymmetry is deliberate: mode,
     * rate and frame size differ from DEVICE to device on a shared bus,
     * while the bit order is a property of the WIRE - a bus with an
     * MSB-first device and an LSB-first one on it is a bus whose two
     * clients disagree about what a byte is, and no engine can arbitrate
     * that. So it is a bus-level verb, applied once, and the Request
     * stays the shape the other two targets carry.
     *
     * @return false when the configuration was refused (it never is,
     * with a valid cached state) - the bus must be IDLE.
     */
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

    /**
     * @brief Begin a transaction (called by SpiBus from main context).
     * @return true when it completed SYNCHRONOUSLY (polled requests, and
     * the degenerate zero-length one); false when it runs on the ISR and
     * a TransferDone will follow. Exactly util/bus_master.hpp's engine
     * contract.
     */
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        in_cmd_ = (r.cmd_len > 0);
        status_ = spi_ok;
        if (total_len() == 0) {
            return true;   // nothing to move: complete on the spot
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
                if (r.len != 0) {
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
        // Polled pump: silence this SPI's own RXNE (a bound handler
        // would steal the frames). Global interrupts STAY ENABLED, so
        // the ticker and everything else preempt this loop freely.
        S::rxne_interrupt(false);
        for (uint8_t i = 0; i < r.cmd_len; ++i) {
            (void)xfer(frame_at(r.cmd.get(), i));
        }
        r.dc.set();   // data phase (a no-op when len == 0)
        for (uint16_t i = 0; i < r.len; ++i) {
            const uint16_t in = xfer(data_frame(i));
            if (r.rx.get() != nullptr) {
                store_frame(r.rx.get(), i, in);
            }
        }
        r.cs.set();   // release: transaction done
        return true;
    }

    /**
     * @brief The instance's interrupt body - call from its vector.
     *
     * Only RXNE is ever armed by this engine (see the class comment),
     * and reading DR is both the capture and the acknowledgement.
     *
     * @return true when the transaction just completed (CS released):
     * the edge on which the app's glue posts TransferDone to the bus AO.
     */
    [[gnu::always_inline]] static bool isr() {
        if ((S::isr() & SpiFlag::rxne) == 0u) {
            return false;
        }
        const uint16_t in = S::data(req_.bits);

        if constexpr (has_engines) {
            if (dma_serves(req_)) {
                // Only the COMMAND phase ever runs on this pump when the
                // engines serve the request: at its end they take the
                // data phase and this interrupt goes quiet. The frame
                // read back is the command's echo - discarded, as the
                // engineless path discards it too.
                (void)in;
                ++pos_;
                if (pos_ >= req_.cmd_len) {
                    in_cmd_ = false;
                    S::rxne_interrupt(false);
                    req_.dc.set();
                    if (req_.len == 0) {
                        req_.cs.set();   // a command-only request: done here
                        return true;
                    }
                    launch_dma();
                    return false;        // dma_isr() is the completion edge
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
            req_.dc.set();   // command phase over
        }
        if (!in_cmd_ && pos_ >= req_.len) {
            S::rxne_interrupt(false);
            req_.cs.set();   // release: transaction done
            return true;
        }
        S::data(req_.bits, next_frame());
        return false;
    }

    /**
     * @brief The DMA channels' interrupt body - call from whichever
     * vector each channel reports on (three vectors serve twelve
     * channels here, so the same call is safe on a shared line: each
     * engine's service() answers for its own channel and returns 0
     * otherwise). Compiles away on an engineless host.
     *
     * A TRANSFER ERROR ON EITHER CHANNEL ENDS THE TRANSACTION with
     * spi_dma_fault: a stopped transmit block starves the receive side
     * for ever, and an error on the receive side means the count can no
     * longer be trusted. Both channels are put away, CS is raised, and
     * the fault is REPORTED rather than retried - retry policy is the
     * bus AO's, not the engine's.
     *
     * @return true when the transaction just completed.
     */
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint8_t tx = TxEngine::service();
            if ((tx & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                return finish_dma(spi_dma_fault);
            }
            if ((tx & TxEngine::flag_complete) != 0u) {
                (void)TxEngine::complete();
                // The transmit side never completes a transaction.
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

    /// The engine's completion status, read by the app glue for the
    /// TransferDone payload. Always spi_ok on an engineless host.
    static uint8_t status() { return status_; }

    /**
     * @brief Put the ENGINE back where start() is legal: engines put
     * away and re-claimed, the peripheral disabled by its own procedure,
     * reconfigured to the applied state and re-enabled, the select
     * window closed. Clocks, pads and the ceiling arithmetic are
     * untouched.
     *
     * The verb a timed SpiBus calls on a transaction that never answered
     * (util/bus_master.hpp) - here that means an ISR-style completion
     * that never posted: a client that never clocked, a lost interrupt,
     * a DMA channel that stopped. The in-flight request's CS is
     * deasserted FIRST: its device sees the transaction end, however
     * garbled, rather than a select held for ever.
     *
     * @return false when a bounded wait ran out (a false engine is
     * refusing, not hanging).
     */
    static bool recover() {
        req_.cs.set();
        if constexpr (has_engines) {
            (void)TxEngine::abandon();   // re-claims by itself
            RxEngine::stop();
            RxEngine::arm(S::data_address(), S::dma_rx_request());
        }
        Nvic::disable(S::irq());
        in_cmd_ = false;
        dma_active_ = false;
        dma_done_ = false;
        S::rxne_interrupt(false);
        // 35.5.9 for a wedged master: the RCC reset is what the chapter
        // itself offers when no sequence applies, and it is the only
        // thing that clears a FIFO the disable procedure cannot drain.
        const bool ok = S::disable();
        S::reset();
        const bool cfg = S::configure(applied_);
        S::enable();
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
        (void)S::disable();
        S::bus_clock(false);
        SckPin::release();
        MosiPin::release();
        if constexpr (pins.miso.valid()) {
            MisoPin::release();
        }
        // The NSS pad was never claimed (see init()), so it is not
        // released either: it belongs to whoever configured it.
    }

    /// Hand the NSS pad to the peripheral - the ONE thing this task does
    /// with it, and only for a program that has deliberately moved the
    /// resource to one of the hardware arrangements (SSOE, NSSP, or the
    /// multi-master input whose low level raises MODF). The task's own
    /// transactions never use it.
    static void claim_nss_pad(bool on) {
        if constexpr (pins.nss.valid()) {
            if (on) {
                NssPin::function(pins.nss.function, {.speed = PinSpeed::very_high});
            } else {
                NssPin::release();
            }
        } else {
            (void)on;
        }
    }

private:
    // The engines carry BYTES: the Request's buffers are bytes, and
    // 8-bit frames are what the DMA path serves (the class comment says
    // why). Checked at THEIR spelling rather than at a pointer mismatch
    // three screens down.
    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio SpiHost: the DMA engines must carry uint8_t elements - the Request's "
         "buffers are bytes and the engined path serves frames of eight bits or less");

    /// Whether the engines serve THIS request: a wide frame is two bytes
    /// per datum and the byte engines cannot express that, so it falls
    /// back to the frame pump.
    static bool dma_serves(const Request& r) { return !spi_frame_is_halfword(r.bits); }

    static void launch_dma() {
        dma_done_ = false;
        dma_active_ = true;
        // The RECEIVE channel goes first: its request rises only when a
        // frame has come back, and the transmit side is what starts the
        // clock.
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

    /// One exit for the data phase, from either flavour of dma_isr().
    /// @return true when the ISR-style caller should post completion.
    static bool finish_dma(uint8_t st) {
        if (st != spi_ok) {
            status_ = st;
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address(), S::dma_rx_request());
        } else if (TxEngine::busy()) {
            // THE RECEIVE BLOCK COMPLETING IS PROOF THE TRANSMIT ONE DID:
            // every frame that came back was clocked out first. Normally
            // the transmit channel's own completion has already been
            // served (its flag rises first, when the last frame reaches
            // the FIFO), but the two channels can report on DIFFERENT
            // vectors and an application that binds only one would leave
            // this engine `busy()` for ever - and the NEXT launch_dma()
            // would then start no transmit block at all. Settling it here
            // costs a branch and makes the slot's contract "bind the
            // vectors your channels report on" instead of "bind both".
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

    /// The polled request's wait on the DMA completion - bounded, like
    /// every wait here (the slowest frame is 256 x 16 PCLK cycles, and
    /// the budget scales with the length).
    static void spin_dma() {
        uint32_t spins = 200'000u + 6'000u * static_cast<uint32_t>(req_.len);
        while (!dma_done_ && spins-- != 0u) {
        }
        if (!dma_done_) {
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address(), S::dma_rx_request());
            dma_active_ = false;
            status_ = spi_dma_fault;
        }
    }

    /// A slower rate is a LARGER BR code, so the ceiling clamps from
    /// below.
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
        c.nss = SpiNss::software;   // the select is a GPIO: see the class comment
        if constexpr (has_engines) {
            c.dma_transmit = true;
            c.dma_receive = true;
        }
        return c;
    }

    /**
     * Put the peripheral where this request wants it. CR1's mode and
     * rate and CR2's frame size are all enable-protected, so any change
     * costs the disable procedure and an enable - which is why the
     * applied state is cached and the pair is paid only when something
     * really moved. A run of requests to one device costs nothing.
     */
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

    /// One polled frame: write, spin on RXNE (which is the frame having
    /// been shifted BOTH ways), read back. Bounded - a bus whose clock
    /// never runs must not hang the kernel - and the bound is generous.
    static uint16_t xfer(uint16_t out) {
        S::data(req_.bits, out);
        uint32_t spins = 200'000u;
        while (!S::rxne() && spins-- != 0u) {
        }
        return S::data(req_.bits);
    }

    static uint16_t total_len() {
        return static_cast<uint16_t>(req_.cmd_len) + req_.len;
    }

    /// A frame out of a byte buffer, by the rule the class comment
    /// states: one byte for 4..8 bits, two bytes low-first for 9..16.
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

    static uint16_t first_frame() {
        return in_cmd_ ? frame_at(req_.cmd.get(), 0) : data_frame(0);
    }
    static uint16_t next_frame() {
        return in_cmd_ ? frame_at(req_.cmd.get(), pos_) : data_frame(pos_);
    }
    static uint16_t data_frame(uint16_t i) {
        return (req_.tx.get() != nullptr) ? frame_at(req_.tx.get(), i) : 0xFFFFu;
    }

    static inline Request req_{};
    static inline uint16_t pos_ = 0;
    static inline bool in_cmd_ = false;
    /// The last completion's status. Plain: written before the
    /// completion edge and read after it.
    static inline uint8_t status_ = spi_ok;
    /// ISR-written, thread-polled (the polled DMA spin): volatile, the
    /// ticker doctrine.
    static inline volatile bool dma_done_ = false;
    static inline volatile bool dma_active_ = false;
    static inline uint8_t rx_sink_ = 0;
    static constexpr uint8_t tx_dummy_ = 0xFF;
    /// The configuration really in the registers - what apply()
    /// compares against and what recover() puts back.
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
 * The other end of the wire: SCK, MOSI and NSS are inputs, the host sets
 * the pace, and the only thing this side controls is WHAT it has ready
 * to shift out when the next clock arrives.
 *
 * THE PUMP MUST RUN ONE AHEAD, and the reason is the FIFO rather than a
 * documented latency. A client answering on RXNE writes its next frame
 * in the gap AFTER a frame has been fully shifted in - by which time the
 * shifter has already taken whatever the TXFIFO held for the frame now
 * starting. So the working shape is: load the first answer while NSS is
 * still high (35.5.8 says so in as many words - "the data register of
 * the slave must already contain data to be sent before starting
 * communication with the master"), park the second at once, and on every
 * received frame write the next-PLUS-ONE. Each value is then in the FIFO
 * a whole frame early. `preload(a, b)` is that opening pair.
 *
 * A CLIENT THAT FALLS BEHIND DOES NOT STALL THE BUS: 35.5.9 is explicit
 * that "there is no underflow error signal for master or slave in SPI
 * mode, and data from the slave is always transacted and processed by
 * the master even if the slave could not prepare it correctly in time".
 * What the master then reads is whatever the shifter held - so a late
 * client is a wrong ANSWER and never a missing one, which is the
 * signature to recognise on the host's side.
 *
 * THE NSS PAD IS THE TRANSACTION. ES0548 2.12.2 makes BSY useless for
 * finding the end of a slave transfer and names the two honest
 * witnesses: the NSS signal, and RXNE. This class reads the pad
 * directly (selected()) and keeps its input buffer on for that reason -
 * the peripheral publishes no select status bit of its own.
 *
 * `drive_output` is the DARK LISTENER of the SAM's client: the MISO pad
 * is handed to the peripheral only for the window this client means to
 * answer in, so a shared harness where something else drives MISO stays
 * uncontested. On a point-to-point link it is simply left on.
 *
 * The surface is polled plus an ISR body, deliberately thin: a client is
 * a protocol and which protocol is the application's.
 *   extern "C" void BRIO_STM32G0_SPI2_HANDLER() { Peer::service(); }
 */
template <uint8_t n, SpiPins pins>
class SpiClient {
    using S = Spi<n>;

    static_assert(spi_pins_valid(pins),
                  "brio SpiClient: these SPI pads are not a link - SCK must be a real "
                  "pad of this device and no two signals may name the same pin");
    static_assert(pins.mosi.valid(),
                  "brio SpiClient: a client must be able to RECEIVE - MOSI is the line "
                  "the host talks on, and a client that cannot hear it has nothing to "
                  "answer");

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

    /// What this end is configured as. Everything a client can be told
    /// that a host cannot; the pads and the role are the task's.
    struct Config {
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;
        bool lsb_first = false;
        /// software: the pad is free and select() drives SSI.
        /// hardware_input: the NSS pad IS the chip select (the default,
        /// and what a real client on a real bus wants).
        SpiNss nss = SpiNss::hardware_input;
        SpiDirection direction = SpiDirection::full_duplex;
        SpiFrameFormat format = SpiFrameFormat::motorola;
        bool crc = false;
        SpiCrcLength crc_length = SpiCrcLength::crc8;
        uint16_t crc_polynomial = 0x0007u;
        /// The baud prescaler means nothing to a Motorola client (the
        /// host's SCK is the clock) - EXCEPT in TI mode, where 35.5.13
        /// makes it control when MISO goes high-impedance at the end of
        /// a transaction. Stated here for that one use.
        SpiClock clock = SpiClock::div16;
        /// Whether the MISO pad is handed to the peripheral at all.
        bool drive_output = true;
    };

    /**
     * @brief Bring the instance up as a client.
     *
     * The APB clock is required even though the SHIFT register runs on
     * the host's SCK: every synchronization in the peripheral crosses
     * into it, and its registers are dead without it.
     *
     * @return false when the configuration is refused.
     */
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        Nvic::disable(S::irq());
        S::bus_clock(true);
        S::reset();
        if (!S::configure(config_of(cfg))) {
            return false;
        }
        // Inputs first, output last: a pad handed over before the
        // peripheral is up would drive whatever the shifter happens to
        // hold onto a bus the host may already be using.
        SckPin::function(pins.sck.function);
        MosiPin::function(pins.mosi.function);
        if constexpr (has_nss_pad) {
            NssPin::function(pins.nss.function);
        }
        drive_output(cfg.drive_output);
        S::flush_rx();
        Nvic::enable(S::irq());
        return true;
    }

    /// Hand the answer line to the peripheral, or take it back. A DARK
    /// listener keeps it false and opens it for exactly the window it
    /// answers in; taking it back leaves the pad in analog, the reset
    /// state, driving nothing.
    static void drive_output(bool on) {
        if constexpr (pins.miso.valid()) {
            if (on) {
                MisoPin::function(pins.miso.function, {.speed = PinSpeed::very_high});
            } else {
                MisoPin::release();
            }
        } else {
            (void)on;
        }
    }

    /// SPE up, and the FIRST TWO ANSWERS in the FIFO before the host's
    /// clock can arrive (35.5.8, and the one-ahead rule in the class
    /// comment). Call it with NSS still high.
    static void enable(uint16_t first = 0xFFFFu, uint16_t second = 0xFFFFu) {
        S::enable();
        write(first);
        write(second);
    }

    /// The disable procedure, through the resource (the client's own
    /// FIFO is drained by it, which is what keeps the NEXT transaction
    /// from reading this one's leftovers).
    static bool disable() { return S::disable(); }

    /// Load the next frame to shift out. Two of these before the first
    /// clock is the one-ahead opening; one per received frame keeps it.
    static void write(uint16_t v) { S::data(bits_, v); }
    static bool writable() { return S::txe(); }

    /// One received frame, or nothing. Reading DR is what clears RXNE,
    /// so this is also the acknowledgement.
    static std::optional<uint16_t> poll() {
        if (!S::rxne()) {
            return {};
        }
        return S::data(bits_);
    }

    /// Is the host holding NSS low right now? A live PAD read - the
    /// peripheral publishes no such status bit, which is why the pad
    /// keeps its input buffer on. Always false without an NSS pad.
    static bool selected() {
        if constexpr (has_nss_pad) {
            return !NssPin::read();
        } else {
            return false;
        }
    }

    /// The software select, for a client whose NSS is SSM/SSI rather
    /// than a pad.
    static void select(bool on) { S::software_select(on); }

    static bool overrun() { return S::overrun(); }
    static void clear_overrun() { S::clear_overrun(); }
    static bool crc_error() { return S::crc_error(); }
    static void clear_crc_error() { S::clear_crc_error(); }
    static bool frame_error() { return S::frame_error(); }
    static void clear_frame_error() { S::clear_frame_error(); }
    static uint32_t status() { return S::status(); }

    /// The ISR body: which sources are raised AND enabled, handed back
    /// for the app's glue to act on. It consumes nothing - only the app
    /// knows where a frame goes.
    [[gnu::always_inline]] static uint32_t isr() { return S::isr(); }

    static void rxne_interrupt(bool on) { S::rxne_interrupt(on); }
    static void error_interrupt(bool on) { S::error_interrupt(on); }

    /// The frame size in force, so poll()/write() pick the right access
    /// width without the caller restating it.
    static SpiDataSize bits() { return bits_; }

    static void release() {
        Nvic::disable(S::irq());
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
        s.clock = c.clock;
        s.bits = c.bits;
        s.lsb_first = c.lsb_first;
        s.nss = c.nss;
        s.direction = c.direction;
        s.format = c.format;
        s.crc = c.crc;
        s.crc_length = c.crc_length;
        s.crc_polynomial = c.crc_polynomial;
        return s;
    }

    static inline SpiDataSize bits_ = SpiDataSize::bits8;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// 35.9.1's BR table, at the 64 MHz this stratum's Clock<> produces.
static_assert(spi_sck_hz(64'000'000UL, SpiClock::div2) == 32'000'000UL);
static_assert(spi_sck_hz(64'000'000UL, SpiClock::div256) == 250'000UL);
static_assert(spi_rate_for(64'000'000UL, 32'000'000UL) == SpiClock::div2);
static_assert(spi_rate_for(64'000'000UL, 31'999'999UL) == SpiClock::div4);
static_assert(spi_rate_for(64'000'000UL, 1'000'000UL) == SpiClock::div64);
static_assert(spi_rate_for(64'000'000UL, 250'000UL) == SpiClock::div256);
// A ceiling below PCLK/256 is REFUSED, never rounded up.
static_assert(!spi_rate_for(64'000'000UL, 249'999UL).has_value());
static_assert(!spi_rate_for(0, 1'000'000UL).has_value());

// Figure 363's two halves.
static_assert(!spi_frame_is_halfword(SpiDataSize::bits8));
static_assert(spi_frame_is_halfword(SpiDataSize::bits9));
static_assert(spi_data_bits(SpiDataSize::bits4) == 4 && spi_data_bits(SpiDataSize::bits16) == 16);
static_assert(spi_frame_mask(SpiDataSize::bits5) == 0x1Fu);
static_assert(spi_data_size_of(4).value() == SpiDataSize::bits4);
static_assert(spi_data_size_of(16).value() == SpiDataSize::bits16);
static_assert(!spi_data_size_of(3).has_value() && !spi_data_size_of(17).has_value());

// 35.7.4's own examples: a 16-bit channel frame with no MCK divides the
// kernel clock by 32 x (2 x I2SDIV + ODD), and with MCK by 256 x that.
static_assert(i2s_sampling_hz(64'000'000UL,
                              I2sConfig{.mode = I2sMode::master_transmit,
                                        .data_length = I2sDataLength::bits16,
                                        .channel_length = I2sChannelLength::bits16,
                                        .divider = 62, .divider_odd = true}) == 16'000UL);
static_assert(i2s_sampling_hz(64'000'000UL,
                              I2sConfig{.mode = I2sMode::master_transmit,
                                        .data_length = I2sDataLength::bits16,
                                        .channel_length = I2sChannelLength::bits32,
                                        .divider = 15, .divider_odd = false}) ==
              33'333UL);
// The chooser is the inverse: 48 kHz on a 16/16 frame at 64 MHz wants
// (2 x div + odd) = 41.66 -> 42, i.e. I2SDIV 21 with ODD clear.
static_assert(i2s_prescaler_for(64'000'000UL, 48'000UL,
                                I2sConfig{.mode = I2sMode::master_transmit})->divider == 21);
static_assert(!i2s_prescaler_for(64'000'000UL, 48'000UL,
                                 I2sConfig{.mode = I2sMode::master_transmit})->divider_odd);
// A divider the register cannot hold is refused rather than clamped.
static_assert(!i2s_prescaler_for(64'000'000UL, 8UL,
                                 I2sConfig{.mode = I2sMode::master_transmit}).has_value());
static_assert(!i2s_config_valid(I2sConfig{.mode = I2sMode::master_transmit, .divider = 1}));
static_assert(i2s_channel_bits(I2sDataLength::bits24, I2sChannelLength::bits16) == 32u);

// The refusals spi_config_valid() owes the chapter.
static_assert(!spi_config_valid(SpiConfig{.bits = SpiDataSize::bits12, .crc = true}));
static_assert(spi_config_valid(SpiConfig{.bits = SpiDataSize::bits16, .crc = true}));
static_assert(!spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0x1020u}));
static_assert(!spi_config_valid(SpiConfig{.mode = SpiMode::mode1, .nss = SpiNss::hardware_pulse}));
static_assert(!spi_config_valid(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));
static_assert(!spi_config_valid(SpiConfig{.nss = SpiNss::hardware_pulse,
                                          .format = SpiFrameFormat::ti}));
static_assert(!spi_config_valid(SpiConfig{.last_dma_transmit_odd = true}));
static_assert(!spi_config_valid(SpiConfig{.bits = SpiDataSize::bits16,
                                          .dma_receive = true,
                                          .last_dma_receive_odd = true}));

} // namespace brio
