/*
 * spi.hpp
 *
 * The SPI of the CH32V203 (RM ch. 20): up to two instances, both roles,
 * 8- or 16-bit frames, the four modes, the three chip-select
 * arrangements, the simplex and bidirectional line modes, hardware CRC
 * and two DMA requests per instance - the STM32F1's SPI under WCH's
 * register names, with one register of its own (HSCR, a high-speed read
 * mode). Three layers, the other strata's arrangement:
 *
 *  - `Spi<n>` is the RESOURCE: the register block, its gate and reset,
 *    the configuration under the rules the chapter states, the data and
 *    status verbs, the sequences that clear each flag, and the ISR body
 *    that reads the raised-and-enabled sources. It decides nothing.
 *  - `SpiHost<n, pins, TxEngine, RxEngine>` is the ENGINE util/
 *    spi_bus.hpp's SpiBus drives: the Request is the other strata's
 *    VERBATIM (chip select, D/C, a command phase, a data phase with
 *    optional out and in, the per-request mode/rate/frame size, the
 *    polled or ISR-pumped completion), so an application written over
 *    SpiBus on a Nucleo runs here unchanged.
 *  - `SpiClient<n, pins>` is the other end of the wire, a thin polled
 *    surface with an ISR body: a client is a protocol, and which one is
 *    the application's.
 *
 * TWO INSTANCES ON TWO BUSES, AND THAT IS THE RATE TABLE. SPI1 sits on
 * PB2, whose rate IS HCLK on this family, and SPI2 on PB1, which the
 * stratum caps at 72 MHz - so the same BR code means two different
 * frequencies on the two instances and a program must ask the INSTANCE
 * (`Spi<n>::bus_hz(clock)`), never the system clock. Which instances a
 * part has is `device::spi_count` (datasheet table 2-1: the 32 KB and
 * 64 KB parts below the CH32V203C8 have SPI1 alone), and `Spi<2>` does
 * not compile where there is one.
 *
 * NO FIFO (figure 20-1): one transmit buffer, one receive buffer, one
 * shift register. TXE means "the buffer moved into the shifter" - a
 * frame EARLY - and RXNE "a frame has been shifted both ways", which is
 * the one moment the bus is provably idle; so the host's pump runs on
 * RXNE and a client answers ONE AHEAD. DFF and CRCEN "can only be
 * written when SPE is 0" (20.4.1), which is what makes apply() pay a
 * disable/enable pair when a request changes the frame size, and BR,
 * LSBFIRST, MSTR, CPOL and CPHA "cannot be modified during
 * communication".
 *
 * ONE OF THE CHAPTER'S SEQUENCES DOES NOT WORK HERE. 20.2.7's recipe
 * for clearing MODF - a STATR access then a CTLR1 write - leaves the
 * flag standing on this silicon under every variant the bench tried
 * (clear_mode_fault()'s own comment lists them), and the block's RCC
 * reset pulse is the way back. The verb reports rather than promises.
 *
 * THE PADS ARE A COLUMN (afio.hpp, table 10-32). SPI1 has two: the
 * default PA4/PA5/PA6/PA7 and code 1's PA15/PB3/PB4/PB5. SPI2 has NO
 * remap field - one column, PB12..PB15, from the datasheet's pin table.
 * `SpiPins` carries the code and init() writes it, refused where the
 * part bonds no pad of it.
 *
 * THE DMA REQUESTS ARE CHANNELS (dma_engine.hpp, table 11-5): SPI1
 * receives on 2 and transmits on 3, SPI2 receives on 4 and transmits on
 * 5. On this family the channel IS the request, so an engine slot
 * naming any other channel is refused at compile time - it would move
 * nothing. IN SLEEP THE BUS MATRIX SERVES THE CORE ALONE (this
 * stratum's finding, docs/ch32v203/README.md): a transport with engines
 * holds the program awake, exactly as usart.hpp's does.
 *
 * WHAT THE CHAPTER GIVES ANOTHER CLASS, AND THIS FILE THEREFORE HAS
 * NOT. HSCR's second bit (HSRXEN2, high-speed read mode 2) is named for
 * the CH32F20x_D8/D8C, CH32V30x_D8/D8C and CH32V31x_D8C alone, and so
 * is BR's alternate ladder (FPCLK/2, /3, /4 ... under HSRXEN): on the
 * CH32V20x_D6 series 20.4.10 says the high-speed read mode "is only
 * valid at clock division 2", so HSRXEN changes the rate table nowhere
 * and `high_speed_read()` refuses at any other BR code. The bit is also
 * WRITE-ONLY here (the register's own access column), so nothing reads
 * it back.
 *
 * I2S. The register file carries the I2S face (SPIx_I2S_CFGR at 0x1C on
 * both instances, SPIx_I2SPR at 0x20 on SPI2 alone - SPI1 has no
 * prescaler and so no master I2S), and chapter 20 is written for four
 * families at once. The CH32V203 DATASHEET gives this series no I2S: no
 * row in table 2-1, no audio signal in any package's pin table, and the
 * clock a master I2S needs (RCC_CFGR2's I2S2SRC and PLL3) is another
 * class's. So this driver carries NO I2S verbs, `i2s_config_writable()`
 * is the one question it answers about that register, and
 * docs/ch32v203/spi.md states the measurement.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "ch32v203/afio.hpp"
#include "ch32v203/clock.hpp"
#include "ch32v203/delay.hpp"
#include "ch32v203/device.hpp"
#include "ch32v203/dma_engine.hpp"
#include "ch32v203/pfic.hpp"
#include "ch32v203/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// =============================================================================
// The registers (tables 20-1 and 20-2): sixteen bits at a four-byte stride
// =============================================================================

struct SpiRegs {
    volatile uint16_t CTLR1;    ///< 0x00
    uint16_t RESERVED0;
    volatile uint16_t CTLR2;    ///< 0x04
    uint16_t RESERVED1;
    volatile uint16_t STATR;    ///< 0x08
    uint16_t RESERVED2;
    volatile uint16_t DATAR;    ///< 0x0c
    uint16_t RESERVED3;
    volatile uint16_t CRCR;     ///< 0x10 the polynomial
    uint16_t RESERVED4;
    volatile uint16_t RCRCR;    ///< 0x14 receive CRC, read-only
    uint16_t RESERVED5;
    volatile uint16_t TCRCR;    ///< 0x18 transmit CRC, read-only
    uint16_t RESERVED6;
    volatile uint16_t I2SCFGR;  ///< 0x1c the I2S face (see the file header)
    uint16_t RESERVED7;
    volatile uint16_t I2SPR;    ///< 0x20 the I2S prescaler, SPI2's alone
    uint16_t RESERVED8;
    volatile uint16_t HSCR;     ///< 0x24 high-speed control
    uint16_t RESERVED9;
};

constexpr uint32_t spi_base_for(uint8_t n) {
    return n == 1 ? pb2_base + 0x3000 : n == 2 ? pb1_base + 0x3800 : 0;
}

/// Which bus an instance hangs on, which is both its gate register and
/// the clock its BR field divides.
constexpr Bus spi_bus_for(uint8_t n) { return n == 1 ? Bus::pb2 : Bus::pb1; }

constexpr uint32_t spi_gate_for(uint8_t n) {
    return n == 1 ? rcc_pb2_spi1 : n == 2 ? rcc_pb1_spi2 : 0;
}

constexpr Irq spi_irq_for(uint8_t n) { return n == 1 ? Irq::spi1 : Irq::spi2; }

/// Does THIS PART have that instance (datasheet table 2-1)?
constexpr bool spi_present(uint8_t n) { return n >= 1u && n <= device::spi_count; }

/// Table 11-5's two rows per instance, read through dma_engine.hpp so
/// that the channel numbers live in exactly one place.
constexpr uint8_t spi_dma_rx_channel(uint8_t n) {
    return dma_request_channel(n == 1 ? DmaRequest::spi1_rx : DmaRequest::spi2_rx);
}
constexpr uint8_t spi_dma_tx_channel(uint8_t n) {
    return dma_request_channel(n == 1 ? DmaRequest::spi1_tx : DmaRequest::spi2_tx);
}

// CTLR1 (20.4.1)
inline constexpr uint16_t spi_cpha     = 1u << 0;
inline constexpr uint16_t spi_cpol     = 1u << 1;
inline constexpr uint16_t spi_mstr     = 1u << 2;
inline constexpr uint16_t spi_br_mask  = 7u << 3;
inline constexpr uint16_t spi_br_shift = 3;
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
// CTLR2 (20.4.2)
inline constexpr uint16_t spi_rxdmaen  = 1u << 0;
inline constexpr uint16_t spi_txdmaen  = 1u << 1;
inline constexpr uint16_t spi_ssoe     = 1u << 2;
inline constexpr uint16_t spi_errie    = 1u << 5;
inline constexpr uint16_t spi_rxneie   = 1u << 6;
inline constexpr uint16_t spi_txeie    = 1u << 7;
/// The three enables as one field, so a verb that rewrites CTLR2 keeps
/// whatever the caller had armed.
inline constexpr uint16_t spi_ctlr2_enables = spi_errie | spi_rxneie | spi_txeie;
// STATR (20.4.3). UDR and CHSID belong to the I2S face and are named
// here because the register has them, not because this driver uses them.
inline constexpr uint16_t spi_rxne     = 1u << 0;
inline constexpr uint16_t spi_txe      = 1u << 1;
inline constexpr uint16_t spi_chside   = 1u << 2;
inline constexpr uint16_t spi_udr      = 1u << 3;
inline constexpr uint16_t spi_crcerr   = 1u << 4;   ///< the one rc_w0 flag of the register
inline constexpr uint16_t spi_modf     = 1u << 5;
inline constexpr uint16_t spi_ovr      = 1u << 6;
inline constexpr uint16_t spi_bsy      = 1u << 7;
// HSCR (20.4.10). Bit 2 (HSRXEN2) is another device class's.
inline constexpr uint16_t spi_hsrxen   = 1u << 0;
// I2S_CFGR (20.4.8): the one bit this stratum ever writes, and only to
// ask the silicon whether the face is there at all.
inline constexpr uint16_t spi_i2smod   = 1u << 11;

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
    /// The three sources ERRIE gates (20.2.8).
    static constexpr uint32_t errors = spi_crcerr | spi_modf | spi_ovr;
};

/// CPOL and CPHA as the four Motorola modes (20.2.1).
enum class SpiMode : uint8_t { mode0 = 0, mode1 = 1, mode2 = 2, mode3 = 3 };

constexpr bool spi_mode_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 2u) != 0u; }
constexpr bool spi_mode_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 1u) != 0u; }

/// BR[2:0]: the INSTANCE'S OWN bus clock over two to the code plus one.
enum class SpiClock : uint8_t {
    div2 = 0, div4 = 1, div8 = 2, div16 = 3, div32 = 4, div64 = 5, div128 = 6, div256 = 7,
};

constexpr uint16_t spi_division(SpiClock c) {
    return static_cast<uint16_t>(2u << static_cast<uint8_t>(c));
}
constexpr uint32_t spi_sck_hz(uint32_t pclk, SpiClock c) {
    return pclk >> (static_cast<uint8_t>(c) + 1u);
}

/// The coarsest code whose SCK does not exceed `max_hz` on a bus at
/// `pclk`; nullopt when even /256 does - refused, never rounded up.
constexpr std::optional<SpiClock> spi_rate_for(uint32_t pclk, uint32_t max_hz) {
    if (pclk == 0u || max_hz == 0u) {
        return {};
    }
    for (uint8_t code = 0; code < 8u; ++code) {
        if ((pclk >> (code + 1u)) <= max_hz) {
            return static_cast<SpiClock>(code);
        }
    }
    return {};
}

/**
 * The BR code for a ceiling as a COMPILE-TIME CONSTANT, refused where
 * the bus cannot make it:
 *
 *     constexpr auto rate = brio::SpiRateOf<SysClock::pclk1_hz, 4'000'000>::clock;
 *
 * which is how a program states a device's datasheet limit and gets the
 * register's own vocabulary back. The runtime chooser spi_rate_for()
 * only REPORTS the same refusal, as an empty optional.
 */
template <uint32_t pclk, uint32_t max_hz>
struct SpiRateOf {
    static_assert(spi_rate_for(pclk, max_hz).has_value(),
                  "brio SPI: this bus cannot produce an SCK at or below that ceiling - even "
                  "its slowest code (the bus clock over 256) is faster, and a rate is never "
                  "rounded up");
    static constexpr SpiClock clock = *spi_rate_for(pclk, max_hz);
};

/// DFF: the two frame sizes this chapter has.
enum class SpiDataSize : uint8_t { bits8 = 0, bits16 = 1 };

constexpr bool spi_frame_is_halfword(SpiDataSize s) { return s == SpiDataSize::bits16; }
constexpr uint8_t spi_data_bits(SpiDataSize s) { return s == SpiDataSize::bits16 ? 16u : 8u; }
constexpr uint16_t spi_frame_mask(SpiDataSize s) {
    return s == SpiDataSize::bits16 ? 0xFFFFu : 0x00FFu;
}

enum class SpiRole : uint8_t { client = 0, host = 1 };

/// The chip-select arrangements of 20.2.1: `software` is SSM/SSI (the
/// pad free, the engine's select an ordinary GPIO); `hardware_output` is
/// SSOE, the pad driven low by the host while SPE is set;
/// `hardware_input` is the multi-host input whose low level raises MODF
/// and demotes the host - and is what frames a CLIENT's transaction.
enum class SpiNss : uint8_t { software, hardware_output, hardware_input };

/// 20.2.4's line arrangements.
enum class SpiDirection : uint8_t {
    full_duplex,       ///< two data lines, both ways
    receive_only,      ///< RXONLY: two lines, the output one released
    half_duplex_out,   ///< BIDIMODE + BIDIOE: one line, transmitting
    half_duplex_in,    ///< BIDIMODE, BIDIOE clear: one line, receiving
};

/// Which pads carry the four signals, and the REMAP CODE that puts the
/// peripheral on them (afio.hpp, table 10-32): init() writes the code.
/// No alternate-function number - a pad has the one function its column
/// gives it. A signal the program does not wire is an invalid Pad.
struct SpiPins {
    Pad nss{};
    Pad sck{};
    Pad miso{};
    Pad mosi{};
    uint8_t remap = 0;
};

/// How many columns an instance has: SPI1's two, and SPI2's one (no
/// remap field exists for it).
constexpr uint8_t spi_column_count(uint8_t n) { return n == 1 ? afio_spi1_codes : 1u; }

/// The pads of column `code` on that instance, every signal named.
constexpr SpiPins spi_pins_for(uint8_t n, uint8_t code) {
    const SpiPadSet p = n == 1 ? afio_spi1_pads(code) : afio_spi2_pad_set;
    return SpiPins{.nss = p.nss, .sck = p.sck, .miso = p.miso, .mosi = p.mosi,
                   .remap = static_cast<uint8_t>(n == 1 ? code : 0u)};
}

/// The reset column of each instance, which is what a program that has
/// not chosen its pads gets.
inline constexpr SpiPins spi1_default_pins = spi_pins_for(1, 0);
inline constexpr SpiPins spi2_default_pins = spi_pins_for(2, 0);

/// The default for `SpiHost<n>` and `SpiClient<n>`: each instance's own
/// first column, chosen by the instance rather than by SPI1's.
template <uint8_t n>
inline constexpr SpiPins spi_default_pins = spi_pins_for(n, 0);

/**
 * Is that column usable on this part, for that instance? A link needs
 * SCK, no two signals may name one pad, the code must be a column the
 * instance has, and every pad the program NAMES must be one the package
 * brings out (afio.hpp's tables are the series'; the bonding is the
 * part's).
 */
constexpr bool spi_pins_valid(uint8_t n, const SpiPins& p) {
    if (!p.sck.valid() || p.remap >= spi_column_count(n)) {
        return false;
    }
    const Pad pads[] = {p.nss, p.sck, p.miso, p.mosi};
    for (uint8_t i = 0; i < 4u; ++i) {
        if (pads[i].valid() && !pad_bonded(pads[i])) {
            return false;
        }
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

/// The refusals the chapter owes: every field must hold a value its
/// register FIELD encodes (a cast-in frame width or BR code would spill
/// into its neighbours); CRC "can only be used in full duplex mode"
/// (20.4.1); a polynomial of zero computes nothing; and SSOE is a host
/// verb ("Disable SS output in master mode") that a client has no use
/// for.
constexpr bool spi_config_valid(const SpiConfig& c) {
    if (static_cast<uint8_t>(c.role) > 1u || static_cast<uint8_t>(c.mode) > 3u ||
        static_cast<uint8_t>(c.clock) > 7u || static_cast<uint8_t>(c.bits) > 1u ||
        static_cast<uint8_t>(c.nss) > 2u || static_cast<uint8_t>(c.direction) > 3u) {
        return false;
    }
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
    v |= static_cast<uint16_t>(static_cast<uint16_t>(c.clock) << spi_br_shift);
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

/// The rate of the bus an INSTANCE hangs on, out of a static or a
/// dynamic clock - the one number its BR field divides.
template <uint8_t instance, typename C>
constexpr uint32_t spi_bus_hz(C clock) {
    if constexpr (C::is_static) {
        (void)clock;
        return instance == 1u ? C::pclk2_hz : C::pclk1_hz;
    } else {
        (void)clock;
        return instance == 1u ? C::pclk2_hz() : C::pclk1_hz();
    }
}

/// The same answer from an HCLK, which is what a dynamic clock hands a
/// user it is about to rebase (the prescalers still hold the rate being
/// left, so this is arithmetic and not a read).
template <uint8_t instance>
constexpr uint32_t spi_bus_hz_at(uint32_t hclk) {
    return instance == 1u ? pclk2_hz_at(hclk) : pclk1_hz_at(hclk);
}

// =============================================================================
// The resource
// =============================================================================

/**
 * Spi<n>: the register block and its verbs.
 *
 *   using Bus = brio::Spi<2>;
 *   Bus::bus_clock(true);
 *   Bus::reset();
 *   Bus::configure({.role = brio::SpiRole::host, .clock = brio::SpiClock::div16});
 *   Bus::enable();
 *
 * `configure()` REFUSES what spi_config_valid() refuses and writes the
 * two control words with SPE clear (DFF and CRCEN are enable-protected,
 * 20.4.1); the interrupt enables are kept as they stand. The ISR body
 * returns the raised-and-enabled sources for the caller to act on; it
 * consumes nothing.
 */
template <uint8_t n>
struct Spi {
    static_assert(spi_base_for(n) != 0, "brio Spi: this family has SPI1 and SPI2");
    static_assert(spi_present(n),
                  "brio Spi: this part does not offer that instance - the parts below the "
                  "CH32V203C8 have SPI1 alone (datasheet table 2-1, parts/<part>.hpp)");

    Spi() = delete;

    static constexpr uint8_t number = n;
    static constexpr Bus bus = spi_bus_for(n);
    static constexpr Irq irq = spi_irq_for(n);
    /// Table 11-5: the two channels this instance's requests reach.
    static constexpr uint8_t dma_rx_channel = spi_dma_rx_channel(n);
    static constexpr uint8_t dma_tx_channel = spi_dma_tx_channel(n);
    /// SPI2 alone carries the I2S prescaler (table 20-2 has it, 20-1
    /// has not), which is what a master I2S would need.
    static constexpr bool has_i2s_prescaler = (n == 2);

    static SpiRegs& regs() { return *reinterpret_cast<SpiRegs*>(spi_base_for(n)); }
    static volatile void* data_address() { return &regs().DATAR; }

    static void bus_clock(bool on) {
        if (on) {
            Rcc::enable(bus, spi_gate_for(n));
        } else {
            Rcc::disable(bus, spi_gate_for(n));
        }
    }
    /// The RCC reset pulse: every register back to its reset value.
    static void reset() { Rcc::reset(bus, spi_gate_for(n)); }

    /// The instance's own bus rate out of the application's clock tag.
    template <typename C>
    static constexpr uint32_t bus_hz(C clock) {
        return spi_bus_hz<n, C>(clock);
    }

    // ---- configuration ------------------------------------------------------

    /// The whole configuration, with SPE clear on the way in. Refused by
    /// the chapter's rules; the interrupt enables are kept as they are.
    static bool configure(const SpiConfig& c) {
        if (!spi_config_valid(c)) {
            return false;
        }
        write_config(c);
        return true;
    }

    /// The same where the configuration is a CONSTANT: what configure()
    /// reports as false is a compile error here, on the line that wrote
    /// the value.
    template <SpiConfig c>
    static void configure() {
        static_assert(spi_config_valid(c),
                      "brio Spi: this configuration is not one chapter 20 has - a field holds "
                      "a value its register field does not encode, or the CRC is asked for "
                      "outside full duplex or with a zero polynomial, or a CLIENT is asked "
                      "for the NSS OUTPUT, which is a host's verb (20.4.2)");
        write_config(c);
    }

    /// Put the column this instance's pads come from into AFIO. SPI2 has
    /// no remap field, so only code 0 is accepted for it.
    static bool remap(uint8_t code) {
        if constexpr (n == 1) {
            return Afio::remap(Remap::spi1, code);
        } else {
            return code == 0u;
        }
    }

    static void enable() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_spe); }
    static bool enabled() { return (regs().CTLR1 & spi_spe) != 0u; }

    /**
     * The drain-then-disable procedure. Clearing SPE while a frame is in
     * flight truncates it on the wire, so this waits for the buffer
     * empty (TXE) and the shifter idle (not BSY) before dropping the
     * bit. Each wait is bounded; false says one ran out and the bit was
     * dropped regardless.
     *
     * A HOST IN RECEIVE-ONLY OR HALF-DUPLEX INPUT NEVER STOPS BY ITSELF:
     * its clock runs as long as SPE stands, so BSY is the wrong thing to
     * wait for there and `disable()` drops the bit at once (the caller
     * stops such a host between frames - see `stop_receive_only()`).
     */
    static bool disable() {
        bool ok = true;
        if (enabled() && !receive_only_host()) {
            ok = wait_until([] { return txe(); }) && wait_until([] { return !busy(); });
        }
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        return ok;
    }

    /// Is this a host whose clock free-runs (RXONLY or BIDIMODE with
    /// BIDIOE clear)? The one configuration whose disable rule differs.
    static bool receive_only_host() {
        const uint16_t c = regs().CTLR1;
        if ((c & spi_mstr) == 0u) {
            return false;
        }
        if ((c & spi_bidimode) != 0u) {
            return (c & spi_bidioe) == 0u;
        }
        return (c & spi_rxonly) != 0u;
    }

    /**
     * Stop a receive-only host cleanly: SPE is dropped in the window
     * between the second-to-last frame's last clock edge and the last
     * frame's first, and the last frame is then read out. The sequence
     * is the F1 lineage's and this chapter does not spell it, so the
     * bench's own finding is what docs/ch32v203/spi.md reports; the verb
     * returns the last frame when one was there.
     */
    static std::optional<uint16_t> stop_receive_only(SpiDataSize bits) {
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        std::optional<uint16_t> last{};
        if (wait_until([] { return rxne(); })) {
            last = data(bits);
        }
        (void)wait_until([] { return !busy(); });
        return last;
    }

    // ---- data ---------------------------------------------------------------

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

    /// What the block is configured as right now - the two questions a
    /// console asks of a live instance.
    static SpiRole role() {
        return (regs().CTLR1 & spi_mstr) != 0u ? SpiRole::host : SpiRole::client;
    }
    static SpiMode mode() {
        const uint16_t c = regs().CTLR1;
        return static_cast<SpiMode>(((c & spi_cpol) != 0u ? 2u : 0u) |
                                    ((c & spi_cpha) != 0u ? 1u : 0u));
    }
    static SpiClock clock() {
        return static_cast<SpiClock>((regs().CTLR1 & spi_br_mask) >> spi_br_shift);
    }
    static SpiDataSize bits() {
        return (regs().CTLR1 & spi_dff) != 0u ? SpiDataSize::bits16 : SpiDataSize::bits8;
    }

    /// Throw away whatever the receive buffer holds and the OVR it may
    /// have raised (the DATAR-then-STATR sequence, 20.2.7).
    static void flush_rx() {
        (void)regs().DATAR;
        (void)regs().STATR;
    }

    // ---- errors (20.2.7) ----------------------------------------------------

    static bool overrun() { return (regs().STATR & spi_ovr) != 0u; }
    /// "Reading the data register followed by the status register will
    /// clear this bit" - DATAR then STATR, in that order.
    static void clear_overrun() {
        (void)regs().DATAR;
        (void)regs().STATR;
    }
    static bool mode_fault() { return (regs().STATR & spi_modf) != 0u; }
    /**
     * 20.2.7's sequence: "first perform a read or write operation to
     * STATR, and then write to CTLR1".
     *
     * MEASURED ON THIS SILICON: IT DOES NOT PUT THE FLAG DOWN. Five
     * variants of those two steps were tried and all leave MODF
     * standing - the CTLR1 write rewriting the register as it stands,
     * the write that RAISES SPE (which is the shape the vendor's own
     * library documents, and the one the chapter's words leave open),
     * the write that restores SSI, the sequence with SSI raised BEFORE
     * it, and a STATR WRITE in place of the read. The only way back
     * measured is the block's RCC reset pulse.
     *
     * So the verb runs the chapter's sequence and REPORTS whether the
     * flag went down, rather than promising what the words say. A host
     * that has been demoted has lost SPE and MSTR with the flag and must
     * be reconfigured anyway: reset() then configure() is that path, and
     * SpiHost::recover() is it written out.
     */
    static bool clear_mode_fault() {
        (void)regs().STATR;
        regs().CTLR1 = regs().CTLR1;
        return !mode_fault();
    }
    static bool crc_error() { return (regs().STATR & spi_crcerr) != 0u; }
    /// The one rc_w0 flag of the register: a zero into its bit, ones
    /// everywhere else.
    static void clear_crc_error() { regs().STATR = static_cast<uint16_t>(~spi_crcerr); }

    // ---- CRC (20.2.5) -------------------------------------------------------

    /// Send the transmit CRC after the frame just written. "This bit
    /// should be set immediately after writing the last data."
    static void crc_next() { regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_crcnext); }
    /// The polynomial, writable with SPE clear like CRCEN itself.
    static void crc_polynomial(uint16_t poly) { regs().CRCR = poly; }
    static uint16_t crc_polynomial() { return regs().CRCR; }
    /// "This register needs to be read when BSY is 0."
    static uint16_t rx_crc() { return regs().RCRCR; }
    static uint16_t tx_crc() { return regs().TCRCR; }

    // ---- the chip select and the rest --------------------------------------

    /// SSI, for a configuration under SSM: high is "not selected" on a
    /// host (and what keeps it a host), low selects a client.
    static void software_select(bool selected) {
        if (selected) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_ssi);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_ssi);
        }
    }

    /// BIDIOE, for a half-duplex link that turns around between phases:
    /// true drives the one data line, false listens on it.
    static void half_duplex_output(bool on) {
        if (on) {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 | spi_bidioe);
        } else {
            regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_bidioe);
        }
    }

    /**
     * HSCR.HSRXEN, the high-speed read mode. On the CH32V20x_D6 series
     * 20.4.10 says it "is only valid at clock division 2", so the verb
     * REFUSES at any other BR code rather than arming a mode the silicon
     * ignores; the bit is write-only here, so nothing reads it back.
     */
    static bool high_speed_read(bool on) {
        if (on && clock() != SpiClock::div2) {
            return false;
        }
        regs().HSCR = on ? spi_hsrxen : uint16_t{0};
        return true;
    }

    /**
     * Does SPI_I2S_CFGR's I2SMOD take a write on this silicon? The
     * datasheet gives this series no I2S (the file header), and this is
     * the one question the driver asks of that register: the bit is set,
     * read back and put away, with SPE clear as 20.4.8 requires. It
     * moves no pad and starts nothing.
     */
    static bool i2s_config_writable() {
        const uint16_t saved = regs().I2SCFGR;
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        regs().I2SCFGR = static_cast<uint16_t>(saved | spi_i2smod);
        const bool took = (regs().I2SCFGR & spi_i2smod) != 0u;
        regs().I2SCFGR = saved;
        return took;
    }

    static void dma_requests(bool tx, bool rx) {
        uint16_t v = static_cast<uint16_t>(regs().CTLR2 & ~(spi_txdmaen | spi_rxdmaen));
        if (tx) { v |= spi_txdmaen; }
        if (rx) { v |= spi_rxdmaen; }
        regs().CTLR2 = v;
    }

    // ---- interrupts (20.2.8) ------------------------------------------------

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
            up |= st & SpiFlag::errors;
        }
        return up;
    }

private:
    /// The two control words, SPE clear on the way in (DFF and CRCEN are
    /// enable-protected, 20.4.1).
    static void write_config(const SpiConfig& c) {
        regs().CTLR1 = static_cast<uint16_t>(regs().CTLR1 & ~spi_spe);
        if (c.crc) {
            regs().CRCR = c.crc_polynomial;
        }
        regs().CTLR1 = spi_ctlr1_of(c);
        regs().CTLR2 =
            static_cast<uint16_t>((regs().CTLR2 & spi_ctlr2_enables) | spi_ctlr2_of(c));
    }

    static void interrupt(uint16_t bit, bool on) {
        if (on) {
            regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 | bit);
        } else {
            regs().CTLR2 = static_cast<uint16_t>(regs().CTLR2 & ~bit);
        }
    }

    /// A bounded wait: true when the condition came, false when the
    /// budget ran out (a bus whose clock never runs must not hang the
    /// kernel; the budget is generous against the slowest frame).
    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = 200'000u; spins != 0u; --spins) {
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
 * THE PUMP RUNS ON RXNE. A frame written to DATAR is clocked out and the
 * frame that comes back raises RXNE when it has been shifted BOTH ways -
 * the one moment the bus is provably idle, and why TXE (raised a frame
 * early, when the buffer moves into the shifter) is never the pump's
 * edge. Per frame: one write, one interrupt, one read.
 *
 * TWO COMPLETION STYLES, the request's choice: `polled` false runs the
 * data on this interrupt with the kernel free between frames (a
 * TransferDone follows), `polled` true spins inside start() with only
 * this instance's own RXNE silenced and completes synchronously.
 *
 * THE ENGINE SLOTS: both or neither (the data phase is full duplex and
 * the transaction's completion is the RECEIVE block's), on the two
 * channels table 11-5 wires to THIS instance and no others. A request in
 * 16-bit frames falls back to the pump: the engines carry bytes.
 *
 * FRAMES IN A BYTE BUFFER: one byte per 8-bit frame, two bytes low-first
 * per 16-bit frame - the other strata's rule.
 *
 * IN SLEEP THE BUS MATRIX SERVES THE CORE ALONE on this family, so a
 * host with engines holds the program awake while a block is in flight;
 * `busy()` is what a sleep site asks.
 */
template <uint8_t n, SpiPins pins = spi_default_pins<n>, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class SpiHost {
    using S = Spi<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from ch32v203/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is "
                  "full-duplex and its completion is the RECEIVE block's");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the two engines must ride two different DMA channels");
    static_assert(!TxEngine::present || dma_engine_channel<TxEngine>() == S::dma_tx_channel,
                  "brio SpiHost: on this family the channel IS the request (RM table 11-5) - "
                  "SPI1 transmits on DMA channel 3 and SPI2 on channel 5, and an engine on "
                  "any other channel would move nothing");
    static_assert(!RxEngine::present || dma_engine_channel<RxEngine>() == S::dma_rx_channel,
                  "brio SpiHost: on this family the channel IS the request (RM table 11-5) - "
                  "SPI1 receives on DMA channel 2 and SPI2 on channel 4, and an engine on "
                  "any other channel would move nothing");
    static_assert(spi_pins_valid(n, pins),
                  "brio SpiHost: these SPI pads are not a link on this part - SCK is required, "
                  "no two signals may name the same pad, the remap code must be a column the "
                  "instance has, and every named pad must be one the package bonds");
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
    static constexpr uint8_t number = n;
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

        /// Per-transaction bus configuration: a shared bus's devices each
        /// name their own, and the engine reprograms the peripheral only
        /// when something CHANGED.
        SpiClock clock = SpiClock::div16;
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;

        /// Completion style: false = per-frame ISR pump, true = POLLED
        /// inside start() (see the class comment).
        bool polled = false;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ----------------------------------------------------------

    /**
     * Bring the instance up as a host: gate, reset, configuration, pads,
     * the PFIC line. `clock` is the app's Clock tag; the rate this
     * instance's BR field divides is ITS OWN BUS'S, which the tag
     * answers. `max_sck_hz` is an optional CEILING for the whole bus,
     * re-resolved on rebase(); 0 = none. False when the ceiling cannot
     * be produced at this clock, or the boot configuration is refused.
     */
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not "
                      "list it among its Users: its SCK ceiling and its cs_setup timing "
                      "would go stale on a clock change");
        Pfic::disable(S::irq);
        ceiling_hz_ = max_sck_hz;
        rebase(clock_hz(clock));
        if (max_sck_hz != 0u && !ceiling_) {
            return false;   // not even the bus over 256 honours it
        }
        S::bus_clock(true);
        S::reset();
        // THE COLUMN IS WRITTEN EVERY TIME, code 0 included: AFIO's field
        // is this instance's alone (no console or crystal shares it), and
        // a program that moves a host back to the reset column would
        // otherwise leave the remap standing.
        if (!S::remap(pins.remap)) {
            return false;
        }
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
        SckPin::function(PinDrive::push_pull, pad_speed_);
        MosiPin::function(PinDrive::push_pull, pad_speed_);
        if constexpr (pins.miso.valid()) {
            MisoPin::input();
        }
        // THE NSS PAD IS NOT CLAIMED: the boot configuration is
        // SpiNss::software and 20.2.1 leaves the pad free under it -
        // which is what this engine's chip select is, an ordinary GPIO
        // the Request carries. claim_nss_pad() is for a program that
        // moves the resource to a hardware arrangement.
        S::enable();
        S::flush_rx();
        Pfic::enable(S::irq);
        return true;
    }

    /// The bus clock changed (DynamicClock fan-out). A Request's `clock`
    /// is a DIVISION of it and scales by itself; what is recomputed is
    /// the ceiling and the cs_setup timing. THE BUS MUST BE IDLE.
    static void rebase(uint32_t hz) {
        hclk_hz_ = hz;
        pclk_hz_ = spi_bus_hz_at<n>(hz);
        ceiling_ = ceiling_hz_ != 0u ? spi_rate_for(pclk_hz_, ceiling_hz_)
                                     : std::optional<SpiClock>{};
        cs_rate_ = delay_rate(hz);
    }

    /// The BR code that produces at most `hz` of SCK on this instance's
    /// bus - the chooser a device's datasheet limit is spoken to.
    static std::optional<SpiClock> clock_for(uint32_t hz) { return spi_rate_for(pclk_hz_, hz); }
    /// What a request at this code really runs at, ceiling included.
    static uint32_t sck_hz(SpiClock c) { return spi_sck_hz(pclk_hz_, clamp(c)); }
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    /// The rate this instance's BR field divides - PCLK2 for SPI1,
    /// PCLK1 for SPI2 - and the HCLK it was derived from.
    static uint32_t reference_hz() { return pclk_hz_; }
    static uint32_t hclk_hz() { return hclk_hz_; }

    /**
     * THE SLEW CLASS OF THE PADS THIS HOST DRIVES (SCK and MOSI; an
     * input pad has none). The fastest edge is also the loudest, and on
     * a bus of jumper wires a data line's edge coupled into the clock is
     * one sampling edge too many - which is a correctness parameter and
     * not a taste, in exactly the modes that sample on the falling edge
     * while the data changes on the rising one (docs/ch32v203/spi.md
     * carries the measurement). The default is `PinSpeed::fast`, and a
     * choice made here SURVIVES the next init().
     */
    static void pad_speed(PinSpeed s) {
        pad_speed_ = s;
        SckPin::function(PinDrive::push_pull, s);
        MosiPin::function(PinDrive::push_pull, s);
    }
    static PinSpeed pad_speed() { return pad_speed_; }

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
            in_cmd_ = false;
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
                        // The command phase runs on the pump; isr() hands
                        // over to the engines at its end.
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
                in_cmd_ = false;
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
        // Polled pump: silence this instance's own RXNE (a bound handler
        // would steal the frames). Global interrupts STAY ENABLED.
        S::rxne_interrupt(false);
        for (uint8_t i = 0; i < r.cmd_len; ++i) {
            (void)xfer(frame_at(r.cmd.get(), i));
        }
        in_cmd_ = false;   // busy() is a question the power model asks
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

    /// The instance's interrupt body - call from its vector. Only RXNE is
    /// ever armed by this engine, and reading DATAR is both the capture
    /// and the acknowledgement. True when the transaction just completed
    /// (CS released): the edge the app's glue posts TransferDone on.
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

    /// Is a transaction in flight? What a sleep site asks on a family
    /// whose bus matrix serves the core alone while it sleeps. A POLLED
    /// request never answers true from outside: it completes inside
    /// start(), on the caller's own stack.
    static bool busy() { return dma_active_ || in_cmd_; }

    /// Put the ENGINE back where start() is legal (the verb a timed
    /// SpiBus calls on a transaction that never answered): the select
    /// released FIRST, the engines put away and re-claimed, the
    /// peripheral reset and reconfigured to the applied state. False when
    /// a bounded wait ran out.
    static bool recover() {
        req_.cs.set();
        if constexpr (has_engines) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address());
        }
        Pfic::disable(S::irq);
        in_cmd_ = false;
        dma_active_ = false;
        dma_done_ = false;
        S::rxne_interrupt(false);
        const bool ok = S::disable();
        S::reset();
        (void)S::remap(pins.remap);
        const bool cfg = S::configure(applied_);
        S::enable();
        S::flush_rx();
        Pfic::enable(S::irq);
        return ok && cfg;
    }

    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Pfic::disable(S::irq);
        (void)S::disable();
        S::bus_clock(false);
        SckPin::release();
        MosiPin::release();
        if constexpr (pins.miso.valid()) {
            MisoPin::release();
        }
    }

    /// Hand the NSS pad to the peripheral - only for a program that has
    /// moved the resource to a hardware arrangement (SSOE's output, or
    /// the multi-host input). The task's own transactions never use it.
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
        // THE REQUESTS ARE ARMED AROUND THE ENGINES AND NOWHERE ELSE: a
        // request that rises while its channel is disabled is latched by
        // the controller and served at the channel's next enable, and the
        // block would come back shifted by one. So RXDMAEN is raised only
        // once the receive channel is enabled, TXDMAEN only once the
        // transmit one is, and finish_dma() drops both.
        // The RECEIVE channel first: its request rises only when a frame
        // has come back, and the transmit side is what starts the clock.
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

    /// One exit for the data phase. True when the ISR-style caller should
    /// post completion.
    static bool finish_dma(uint8_t st) {
        S::dma_requests(false, false);
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
    /// slowest frame is 256 x 16 bus cycles; the budget scales).
    static void spin_dma() {
        uint32_t spins = 200'000u + 6'000u * static_cast<uint32_t>(req_.len);
        while (!dma_done_ && spins-- != 0u) {
        }
        if (!dma_done_) {
            S::dma_requests(false, false);
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
        // The DMA requests are NOT part of the applied configuration:
        // launch_dma() raises them around the engines (see there).
        return c;
    }

    /// Put the peripheral where this request wants it. DFF is
    /// enable-protected and BR/CPOL/CPHA "cannot be modified during
    /// communication", so any change costs a disable/enable pair - paid
    /// only when something moved.
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
        uint32_t spins = 400'000u;
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
    static uint16_t next_frame() {
        return in_cmd_ ? frame_at(req_.cmd.get(), pos_) : data_frame(pos_);
    }
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
    static inline uint32_t pclk_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<SpiClock> ceiling_{};
    static inline DelayRate cs_rate_{};
    static inline PinSpeed pad_speed_ = PinSpeed::fast;
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
 * ONE FRAME AHEAD. With no FIFO the transmit buffer holds exactly the
 * next answer: 20.2.3 says the slave starts to transmit on the first
 * sampling edge, moving the buffer into the shifter and raising TXE -
 * the moment to write the NEXT one. So: load the first answer before the
 * host selects (`enable(first)`), then write frame k + 1 on TXE while
 * frame k shifts; a write made on RXNE instead reaches the buffer one
 * frame late under a host that clocks back to back. A client that falls
 * behind does not stall the bus: what the host reads is whatever the
 * shifter held, a wrong answer and never a missing one.
 *
 * THE NSS PAD IS THE TRANSACTION: read directly (selected()), since the
 * peripheral publishes no select status. `drive_output` makes this client
 * a DARK LISTENER - MISO is handed to the peripheral only for the window
 * it answers in, so a shared harness stays uncontested. On this family
 * the pad can be taken back to a floating INPUT with the block still
 * enabled (`drive_output(false)` at any time), which is what the bench
 * measured; nothing of the peripheral has to move with it.
 */
template <uint8_t n, SpiPins pins = spi_default_pins<n>>
class SpiClient {
    using S = Spi<n>;

    static_assert(spi_pins_valid(n, pins),
                  "brio SpiClient: these SPI pads are not a link on this part - SCK is "
                  "required, no two signals may name the same pad, the remap code must be a "
                  "column the instance has, and every named pad must be one the package bonds");
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
    static constexpr uint8_t number = n;
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

    /// Bring the instance up as a client. The bus clock is required even
    /// though the shifter runs on the host's SCK. False when the
    /// configuration is refused - a client with SSOE, above all, which is
    /// a host's verb.
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        Pfic::disable(S::irq);
        S::bus_clock(true);
        S::reset();
        if (!S::remap(pins.remap)) {
            return false;
        }
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
        Pfic::enable(S::irq);
        return true;
    }

    /// Hand the answer line to the peripheral, or take it back (a
    /// floating input, driving nothing) - the dark listener's one verb.
    static void drive_output(bool on) {
        if constexpr (pins.miso.valid()) {
            if (on) {
                MisoPin::function(PinDrive::push_pull, pad_speed_);
            } else {
                MisoPin::release();
            }
        } else {
            (void)on;
        }
    }
    static bool output_driven() {
        if constexpr (pins.miso.valid()) {
            return MisoPin::nibble() ==
                   pin_nibble(PinMode::alternate, PinDrive::push_pull, pad_speed_);
        } else {
            return false;
        }
    }

    /// The slew class of the ONE pad a client drives - SpiHost's verb
    /// with the same reason behind it, and a choice that survives the
    /// next init().
    static void pad_speed(PinSpeed s) {
        pad_speed_ = s;
        if (output_driven_now()) {
            drive_output(true);
        }
    }
    static PinSpeed pad_speed() { return pad_speed_; }

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
        Pfic::disable(S::irq);
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

    static bool output_driven_now() {
        if constexpr (pins.miso.valid()) {
            return (MisoPin::nibble() & 0xCu) == 0x8u;   // CNF 10: alternate push-pull
        } else {
            return false;
        }
    }

    static inline SpiDataSize bits_ = SpiDataSize::bits8;
    static inline PinSpeed pad_speed_ = PinSpeed::fast;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// 20.4.1's BR table on the two buses of the 144 MHz tree: SPI1 divides
// PCLK2 (= HCLK) and SPI2 divides PCLK1 (72 MHz), so the same code is
// two frequencies.
static_assert(spi_sck_hz(144'000'000UL, SpiClock::div2) == 72'000'000UL);
static_assert(spi_sck_hz(72'000'000UL, SpiClock::div2) == 36'000'000UL);
static_assert(spi_sck_hz(144'000'000UL, SpiClock::div256) == 562'500UL);
static_assert(spi_sck_hz(72'000'000UL, SpiClock::div256) == 281'250UL);
static_assert(spi_division(SpiClock::div2) == 2 && spi_division(SpiClock::div256) == 256);
static_assert(spi_rate_for(144'000'000UL, 72'000'000UL) == SpiClock::div2);
static_assert(spi_rate_for(144'000'000UL, 71'999'999UL) == SpiClock::div4);
static_assert(spi_rate_for(72'000'000UL, 1'000'000UL) == SpiClock::div128);
static_assert(spi_rate_for(144'000'000UL, 562'500UL) == SpiClock::div256);
// A ceiling below the bus over 256 is REFUSED, never rounded up.
static_assert(!spi_rate_for(144'000'000UL, 562'499UL).has_value());
static_assert(!spi_rate_for(0, 1'000'000UL).has_value());

static_assert(!spi_frame_is_halfword(SpiDataSize::bits8) &&
              spi_frame_is_halfword(SpiDataSize::bits16));
static_assert(spi_frame_mask(SpiDataSize::bits8) == 0xFFu && spi_data_bits(SpiDataSize::bits16) == 16);

// The four modes.
static_assert(!spi_mode_cpol(SpiMode::mode1) && spi_mode_cpha(SpiMode::mode1));
static_assert(spi_mode_cpol(SpiMode::mode2) && !spi_mode_cpha(SpiMode::mode2));

// The two buses, their gates and their vectors.
static_assert(spi_base_for(1) == 0x40013000UL && spi_base_for(2) == 0x40003800UL);
static_assert(spi_bus_for(1) == Bus::pb2 && spi_bus_for(2) == Bus::pb1);
static_assert(spi_gate_for(1) == rcc_pb2_spi1 && spi_gate_for(2) == rcc_pb1_spi2);
static_assert(spi_irq_for(1) == Irq::spi1 && spi_irq_for(2) == Irq::spi2);
static_assert(spi_bus_hz_at<1>(144'000'000UL) == 144'000'000UL);
static_assert(spi_bus_hz_at<2>(144'000'000UL) == 72'000'000UL);

// Table 11-5's four rows, through dma_engine.hpp.
static_assert(spi_dma_rx_channel(1) == 2 && spi_dma_tx_channel(1) == 3);
static_assert(spi_present(2) == (device::spi_count >= 2u));

// Table 10-32's columns, and SPI2's single one.
static_assert(spi_column_count(1) == 2 && spi_column_count(2) == 1);
static_assert(spi_pins_for(1, 0).sck == Pad{'A', 5} && spi_pins_for(1, 0).nss == Pad{'A', 4});
static_assert(spi_pins_for(1, 1).sck == Pad{'B', 3} && spi_pins_for(1, 1).nss == Pad{'A', 15} &&
              spi_pins_for(1, 1).remap == 1u);
static_assert(spi_pins_for(2, 0).sck == Pad{'B', 13} && spi_pins_for(2, 0).mosi == Pad{'B', 15} &&
              spi_pins_for(2, 0).remap == 0u);

// The refusals spi_pins_valid() owes: a column the instance has not, two
// signals on one pad, and no SCK.
static_assert(!spi_pins_valid(2, SpiPins{.sck = {'B', 13}, .mosi = {'B', 15}, .remap = 1}));
static_assert(!spi_pins_valid(1, SpiPins{.sck = {'A', 5}, .mosi = {'A', 5}}));
static_assert(!spi_pins_valid(1, SpiPins{.mosi = {'A', 7}}));

// A value no register field encodes is refused too.
static_assert(!spi_config_valid(SpiConfig{.bits = static_cast<SpiDataSize>(2)}));
static_assert(!spi_config_valid(SpiConfig{.clock = static_cast<SpiClock>(8)}));
static_assert(!spi_config_valid(SpiConfig{.nss = static_cast<SpiNss>(3)}));

// The compile-time chooser and its refusal.
static_assert(SpiRateOf<72'000'000UL, 4'000'000UL>::clock == SpiClock::div32);
static_assert(SpiRateOf<144'000'000UL, 562'500UL>::clock == SpiClock::div256);

// The refusals spi_config_valid() owes the chapter.
static_assert(spi_config_valid(SpiConfig{}));
static_assert(!spi_config_valid(SpiConfig{.direction = SpiDirection::half_duplex_out, .crc = true}));
static_assert(!spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0}));
static_assert(!spi_config_valid(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));

// The control words: a mode 3 host at /16 under software select, and a
// hardware-output select with the receive request armed.
static_assert(spi_ctlr1_of(SpiConfig{.mode = SpiMode::mode3, .clock = SpiClock::div16}) ==
              (spi_cpha | spi_cpol | spi_mstr | (3u << spi_br_shift) | spi_ssm | spi_ssi));
static_assert(spi_ctlr2_of(SpiConfig{.nss = SpiNss::hardware_output, .dma_receive = true}) ==
              (spi_ssoe | spi_rxdmaen));
// A client under SSM keeps SSI low; the line modes are two bits.
static_assert(spi_ctlr1_of(SpiConfig{.role = SpiRole::client, .clock = SpiClock::div2}) == spi_ssm);
static_assert((spi_ctlr1_of(SpiConfig{.direction = SpiDirection::half_duplex_in}) & spi_bidimode) != 0u);
static_assert((spi_ctlr1_of(SpiConfig{.direction = SpiDirection::receive_only}) & spi_rxonly) != 0u);

}  // namespace brio
