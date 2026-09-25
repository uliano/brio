/*
 * spi.hpp
 *
 * The SPI of the CH32V203 and the CH32V303 (RM ch. 20): up to three
 * instances, both roles, 8- or 16-bit frames, the four modes, the three
 * chip-select arrangements, the simplex and bidirectional line modes,
 * hardware CRC and two DMA requests per instance - the STM32F1's SPI under
 * WCH's register names, with one register of its own (HSCR, a high-speed
 * read mode) - and on the CH32V303RC and VC the I2S FACE of SPI2 and SPI3.
 * Four layers, the other strata's arrangement:
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
 *  - `I2s<n>` is the same register block in its AUDIO face, a resource
 *    and nothing above it: the four standards, the data and channel
 *    widths, both roles and both directions, the clock generator and its
 *    master clock, the flags and the requests the two faces share.
 *
 * INSTANCES ON TWO BUSES, AND THAT IS THE RATE TABLE. SPI1 sits on PB2,
 * whose rate IS HCLK on this family, and SPI2 and SPI3 on PB1, which the
 * stratum caps at 72 MHz - so the same BR code means two different
 * frequencies on SPI1 and on the other two, and a program must ask the
 * INSTANCE (`Spi<n>::bus_hz(clock)`), never the system clock. Which
 * instances a part has is `device::spi_instances` (the datasheets' tables
 * 2-1 and 2-1-1: the parts below the CH32V203C8 have SPI1 alone, the
 * CH32V203C8 and RB and the 128 KB CH32V303 SPI1 and SPI2, the CH32V303RC
 * and VC all three), and an instance a part has not got does not compile.
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
 * THE PADS ARE A COLUMN (afio.hpp, tables 10-32 and 10-33). SPI1 has two:
 * the default PA4/PA5/PA6/PA7 and code 1's PA15/PB3/PB4/PB5. SPI2 has NO
 * remap field - one column, PB12..PB15, from the datasheet's pin table.
 * SPI3 has two: the default PA15/PB3/PB4/PB5 - the very pads of SPI1's
 * second column - and code 1's PA4/PC10/PC11/PC12, behind PCFR1's bit 28,
 * which moves I2S3's pads with them. `SpiPins` carries the code and
 * init() writes it, refused where the part bonds no pad of it.
 *
 * THE DMA REQUESTS ARE SLOTS (dma_engine.hpp, tables 11-2 to 11-5): SPI1
 * receives on DMA1's channel 2 and transmits on 3, SPI2 on DMA1's 4 and
 * 5, and SPI3 - and the I2S face of SPI3, whose requests are the same two
 * - on DMA2's 1 and 2. On this family the channel IS the request, so an
 * engine slot naming any other one is refused at compile time - it would
 * move nothing. IN SLEEP THE BUS MATRIX SERVES THE CORE ALONE (this
 * stratum's finding, docs/ch32vx03/README.md): a transport with engines
 * holds the program awake, exactly as usart.hpp's does.
 *
 * THE HIGH-SPEED READ MODE IS A LOT'S STORY, AND ONE CODE MEANS THE SAME
 * ON EVERY LOT. HSCR.HSRXEN delays a HOST'S sampling of MISO so that a
 * late answer is caught at the top rate. 20.4.10 tells it four ways by
 * class and lot: on the CH32V20x_D6 it "is only valid at clock division
 * 2" and the bit is write-only; on the CH32V20x_D8 the bit is write-only
 * and the mode valid at /2 alone on lots whose fifth digit from the end
 * is below 2, anywhere on the others; on the CH32V30x_D8 lots whose
 * penultimate sixth digit is not zero the bit reads back, BR takes a
 * second ladder under it (20.4.1: FPCLK/2, /3 ... /9) and a second mode,
 * HSRXEN2, exists; on that class's other lots the bit is write-only again
 * and the mode valid at /2 alone where the fifth digit is below 2. BR =
 * 000 is FPCLK/2 in BOTH ladders, so the one code at
 * which HSRXEN means the same thing on every die of every class is /2 -
 * and `high_speed_read()` refuses at any other, at run time against the
 * code in the register and at compile time where the configuration is a
 * constant (`configure_high_speed_read<cfg>()`). HSRXEN2 is a verb that
 * asks the die (the CH32V303's, and only with SPI's bus at 120 MHz or
 * above, which only SPI1's PB2 reaches). What the mode buys is measured
 * (docs/ch32vx03/spi.md): on the CH32V303VCT6, SPI2 hosting SPI3 over
 * the evaluation board's wires at /2 - 36 MHz - read 254 bytes of 256
 * wrong without it, 253 of them the client's byte ONE BIT LATE, and every
 * byte right with it; that die kept no HSRXEN2.
 *
 * THE I2S FACE, THE CH32V303RC'S AND VC'S. The register file carries
 * SPIx_I2S_CFGR on every instance and SPIx_I2SPR on SPI2 and SPI3, and
 * chapter 20 is written for four families at once; what decides whether
 * a part HAS the face is its datasheet - table 2-1-1 gives two I2S to the
 * 256 KB CH32V303 and none to the 128 KB ones, and the CH32V203's table
 * 2-1 has no I2S row at all - so `I2s<n>` compiles where
 * `device::has_i2s` says and for SPI2 and SPI3 alone, and the SPI face
 * keeps `i2s_config_writable()`, the one question it asks of that
 * register elsewhere. THE CLOCK A MASTER DIVIDES IS SYSCLK: figure 3-3,
 * the simple tree this class has, draws SYSCLK to both I2S interfaces,
 * and RCC_CFGR2's I2S2SRC and I2S3SRC only choose between SYSCLK and a
 * PLL3 the CH32V30x_D8C has and this class has not - so `i2s_clock_hz()`
 * answers a static Clock's SYSCLK, and the divider pair is 20.3.3's
 * arithmetic against it (measured: 16015 frames a second counted against
 * the core for the 16014 that arithmetic gives at 144 MHz). The face
 * shares the vector, the gate, the reset, CTLR2's enables and the two DMA
 * requests with the SPI face; CTLR1, the CRC and MODF are not used in
 * it. ONE SENTENCE OF 20.3.6.2 DOES NOT HOLD: TXE reads 1 on a configured
 * transmitting face with I2SE clear, and a datum written there is the
 * first word on the wire once the face is enabled (measured on the
 * CH32V303VCT6) - so nothing here waits for TXE to rise at the enable,
 * and a program may load its first word before it.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "ch32vx03/afio.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/delay.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/dma_engine.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
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

/// The three instances this family addresses (tables 20-1 to 20-3):
/// SPI1 on PB2, SPI2 and SPI3 on PB1 - device.hpp's spi3_base.
constexpr uint32_t spi_base_for(uint8_t n) {
    return n == 1 ? pb2_base + 0x3000 : n == 2 ? pb1_base + 0x3800 : n == 3 ? spi3_base : 0;
}

/// Which bus an instance hangs on, which is both its gate register and
/// the clock its BR field divides.
constexpr Bus spi_bus_for(uint8_t n) { return n == 1 ? Bus::pb2 : Bus::pb1; }

constexpr uint32_t spi_gate_for(uint8_t n) {
    return n == 1 ? rcc_pb2_spi1 : n == 2 ? rcc_pb1_spi2 : n == 3 ? rcc_pb1_spi3 : 0;
}

/// The instance's vector - SPI3's is entry 67 of the CH32V303's own tail,
/// and irq_none on a class that has no such line.
constexpr Irq spi_irq_for(uint8_t n) {
    return n == 1 ? Irq::spi1 : n == 2 ? Irq::spi2 : Irq::spi3;
}

/// Does THIS PART have that instance? The part table's mask
/// (device::spi_instances, from the datasheets' tables 2-1 and 2-1-1),
/// never a count read as "the first n".
constexpr bool spi_present(uint8_t n) {
    return n >= 1u && n <= 3u && (device::spi_instances & static_cast<uint16_t>(1U << n)) != 0u;
}

/// The two rows per instance (tables 11-2 to 11-5), read through
/// dma_engine.hpp so that the slots live in exactly one place - SPI3's
/// are DMA2's - and the channel numbers of those slots, which is what a
/// message prints.
constexpr DmaSlot spi_dma_rx_slot(uint8_t n) {
    return dma_request_channel(n == 1 ? DmaRequest::spi1_rx
                               : n == 2 ? DmaRequest::spi2_rx : DmaRequest::spi3_rx);
}
constexpr DmaSlot spi_dma_tx_slot(uint8_t n) {
    return dma_request_channel(n == 1 ? DmaRequest::spi1_tx
                               : n == 2 ? DmaRequest::spi2_tx : DmaRequest::spi3_tx);
}
constexpr uint8_t spi_dma_rx_channel(uint8_t n) { return spi_dma_rx_slot(n).channel; }
constexpr uint8_t spi_dma_tx_channel(uint8_t n) { return spi_dma_tx_slot(n).channel; }

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
// HSCR (20.4.10). HSRXEN is every class's; HSRXEN2 is the CH32V30x_D8's,
// and there a lot's (the file header).
inline constexpr uint16_t spi_hsrxen   = 1u << 0;
inline constexpr uint16_t spi_hsrxen2  = 1u << 2;
/// HSRXEN2 "is used only when ... SPI PCLK is greater than or equal to
/// 120M" (20.4.10): the bus rate below which its verb refuses.
inline constexpr uint32_t spi_hsrxen2_min_bus_hz = 120'000'000UL;
// SPIx_I2S_CFGR (20.4.8): the face's whole configuration, and I2SMOD the
// one bit the SPI face writes, only to ask the silicon whether the
// register answers.
inline constexpr uint16_t spi_i2s_chlen        = 1u << 0;
inline constexpr uint16_t spi_i2s_datlen_shift = 1;
inline constexpr uint16_t spi_i2s_datlen_mask  = 3u << 1;
inline constexpr uint16_t spi_i2s_ckpol        = 1u << 3;
inline constexpr uint16_t spi_i2s_std_shift    = 4;
inline constexpr uint16_t spi_i2s_std_mask     = 3u << 4;
inline constexpr uint16_t spi_i2s_pcmsync      = 1u << 7;
inline constexpr uint16_t spi_i2s_cfg_shift    = 8;
inline constexpr uint16_t spi_i2s_cfg_mask     = 3u << 8;
inline constexpr uint16_t spi_i2se             = 1u << 10;
inline constexpr uint16_t spi_i2smod           = 1u << 11;
// SPIx_I2SPR (20.4.9), SPI2's and SPI3's: the linear divider, ODD and the
// master clock's output enable.
inline constexpr uint16_t spi_i2s_div_mask = 0x00FFu;
inline constexpr uint16_t spi_i2s_odd      = 1u << 8;
inline constexpr uint16_t spi_i2s_mckoe    = 1u << 9;

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
    /// The three sources ERRIE gates in the SPI face (20.2.8).
    static constexpr uint32_t errors = spi_crcerr | spi_modf | spi_ovr;
    /// The I2S face's own two: a client transmitter clocked before its
    /// data register was written, and the side the datum belongs to.
    static constexpr uint32_t underrun = spi_udr;
    static constexpr uint32_t channel_side = spi_chside;
    /// The two sources ERRIE gates in the I2S face (20.3.8).
    static constexpr uint32_t i2s_errors = spi_ovr | spi_udr;
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

/// How many columns an instance has: SPI1's two, SPI2's one (no remap
/// field exists for it) and SPI3's two (table 10-33).
constexpr uint8_t spi_column_count(uint8_t n) {
    return n == 1 ? afio_spi1_codes : n == 3 ? afio_spi3_codes : 1u;
}

/// The pads of column `code` on that instance, every signal named.
constexpr SpiPins spi_pins_for(uint8_t n, uint8_t code) {
    const SpiPadSet p = n == 1   ? afio_spi1_pads(code)
                        : n == 3 ? afio_spi3_pads(code)
                                 : afio_spi2_pad_set;
    return SpiPins{.nss = p.nss, .sck = p.sck, .miso = p.miso, .mosi = p.mosi,
                   .remap = static_cast<uint8_t>(n == 2 ? 0u : code)};
}

/// The reset column of each instance, which is what a program that has
/// not chosen its pads gets.
inline constexpr SpiPins spi1_default_pins = spi_pins_for(1, 0);
inline constexpr SpiPins spi2_default_pins = spi_pins_for(2, 0);
inline constexpr SpiPins spi3_default_pins = spi_pins_for(3, 0);

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
    static_assert(spi_base_for(n) != 0, "brio Spi: this family has SPI1, SPI2 and SPI3");
    static_assert(spi_present(n),
                  "brio Spi: this part does not offer that instance - the parts below the "
                  "CH32V203C8 have SPI1 alone, and SPI3 is the CH32V303RC's and VC's (the "
                  "datasheets' tables 2-1 and 2-1-1, parts/<part>.hpp)");

    Spi() = delete;

    static constexpr uint8_t number = n;
    static constexpr Bus bus = spi_bus_for(n);
    static constexpr Irq irq = spi_irq_for(n);
    /// Tables 11-2 to 11-5: the two slots this instance's requests reach.
    static constexpr DmaSlot dma_rx_slot = spi_dma_rx_slot(n);
    static constexpr DmaSlot dma_tx_slot = spi_dma_tx_slot(n);
    static constexpr uint8_t dma_rx_channel = dma_rx_slot.channel;
    static constexpr uint8_t dma_tx_channel = dma_tx_slot.channel;
    /// SPI2 and SPI3 carry the I2S prescaler (tables 20-2 and 20-3 have
    /// it, 20-1 has not), which is what a master I2S needs; whether the
    /// PART has the I2S face at all is device::has_i2s.
    static constexpr bool has_i2s_prescaler = (n != 1);

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
    /// no remap field, so only code 0 is accepted for it; SPI3's field
    /// (PCFR1 bit 28) moves I2S3's pads with its own.
    static bool remap(uint8_t code) {
        if constexpr (n == 1) {
            return Afio::remap(Remap::spi1, code);
        } else if constexpr (n == 3) {
            return Afio::remap(Remap::spi3, code);
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
     * bench's own finding is what docs/ch32vx03/spi.md reports; the verb
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
     * HSCR.HSRXEN, the high-speed read mode: a HOST's sampling of MISO
     * moved late enough to catch an answer at the top rate. The verb
     * REFUSES at any BR code but /2 - the one code whose meaning no lot
     * of any class changes (the file header) - rather than arming a mode
     * a die may ignore or read on another ladder. The bit is write-only
     * on every die but the CH32V303 lots that also have HSRXEN2, so
     * nothing reads it back, and HSCR is written WHOLE: HSRXEN2 goes with
     * the mode it adds to. The mode outlives a reconfiguration - HSCR is
     * not CTLR1 - so a host that moves off /2 turns it off first.
     */
    static bool high_speed_read(bool on) {
        if (on && clock() != SpiClock::div2) {
            return false;
        }
        regs().HSCR = on ? spi_hsrxen : uint16_t{0};
        return true;
    }

    /// The whole configuration AND the high-speed read, where the
    /// configuration is a CONSTANT: a BR code other than /2, or a client
    /// (the mode is a host's reading), is a compile error on the line
    /// that wrote the value - configure<cfg>()'s refusals besides.
    template <SpiConfig c>
    static void configure_high_speed_read() {
        static_assert(spi_config_valid(c),
                      "brio Spi: this configuration is not one chapter 20 has (see configure<cfg>())");
        static_assert(c.clock == SpiClock::div2,
                      "brio Spi: the high-speed read mode means the same thing on every lot at BR "
                      "= /2 alone (20.4.10: the CH32V20x_D6's only valid code, and the one the "
                      "CH32V30x_D8's second ladder leaves where it was)");
        static_assert(c.role == SpiRole::host,
                      "brio Spi: the high-speed read mode moves a HOST's sampling of MISO "
                      "(20.4.10) - a client samples on its host's clock");
        write_config(c);
        regs().HSCR = spi_hsrxen;
    }

    /**
     * HSCR.HSRXEN2, the second high-speed read mode: the CH32V30x_D8's,
     * on the lots 20.4.10's note names, "used only when HSRXEN is turned
     * on and SPI PCLK is greater than or equal to 120M". So the verb takes
     * the instance's bus rate, refuses below 120 MHz - which leaves it to
     * SPI1, PB1 being capped at 72 - and at a BR code other than /2, and
     * writes the two modes TOGETHER, mode 2 being an addition to mode 1.
     * It then ASKS THE DIE, the way this stratum asks for a lot's bit: the
     * word is read FIRST - HSCR holds nothing outside its two bits, so a
     * word with any other bit set is another register's answer (an
     * address that mirrors another has been met on this die) and the
     * answer is no with nothing written - then the two bits are written
     * and the word must come back as exactly those two, which is what the
     * lots that have HSRXEN2 give (HSRXEN reads back there too). A die
     * that did not keep them is left at HSRXEN alone and the answer is
     * false. Refused at compile time on the CH32V203's classes.
     */
    static bool high_speed_read2(bool on, uint32_t bus_hz) {
        static_assert(device::device_class == DeviceClass::v30x_d8,
                      "brio Spi: HSRXEN2 is the CH32V30x_D8's (20.4.10's note) - no CH32V203 has "
                      "it");
        if (!on) {
            regs().HSCR = spi_hsrxen;
            return true;
        }
        if (bus_hz < spi_hsrxen2_min_bus_hz || clock() != SpiClock::div2) {
            return false;
        }
        constexpr uint16_t both = static_cast<uint16_t>(spi_hsrxen | spi_hsrxen2);
        if ((regs().HSCR & static_cast<uint16_t>(~both)) != 0u) {
            return false;   // the read path is not HSCR's own
        }
        regs().HSCR = both;
        if (regs().HSCR != both) {
            regs().HSCR = spi_hsrxen;
            return false;   // this die's lot has no HSRXEN2
        }
        return true;
    }

    /**
     * Does SPI_I2S_CFGR's I2SMOD take a write on this silicon? On a part
     * the datasheet gives no I2S (the file header) this is the one
     * question the driver asks of that register; where the face is the
     * part's, `I2s<n>` is its driver. The bit is set, read back and put
     * away, with SPE clear as 20.4.8 requires. It moves no pad and starts
     * nothing.
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
                  "DmaRxEngine from ch32vx03/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is "
                  "full-duplex and its completion is the RECEIVE block's");
    static_assert(dma_engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the two engines must ride two different DMA channels");
    static_assert(!TxEngine::present || dma_engine_slot<TxEngine>() == S::dma_tx_slot,
                  "brio SpiHost: on this family the channel IS the request (RM tables 11-2 to "
                  "11-5) - SPI1 transmits on DMA1's channel 3, SPI2 on DMA1's 5 and SPI3 on "
                  "DMA2's 2, and an engine on any other slot would move nothing");
    static_assert(!RxEngine::present || dma_engine_slot<RxEngine>() == S::dma_rx_slot,
                  "brio SpiHost: on this family the channel IS the request (RM tables 11-2 to "
                  "11-5) - SPI1 receives on DMA1's channel 2, SPI2 on DMA1's 4 and SPI3 on "
                  "DMA2's 1, and an engine on any other slot would move nothing");
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
     * while the data changes on the rising one (docs/ch32vx03/spi.md
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
// The I2S face (20.3, 20.4.8, 20.4.9) - the CH32V303RC's and VC's
// =============================================================================

/// I2SCFG[1:0] (20.4.8): what the block is on the audio bus - a host
/// drives CK and WS, a client takes them - and which way SD goes.
enum class I2sMode : uint8_t {
    client_transmit = 0,
    client_receive = 1,
    host_transmit = 2,
    host_receive = 3,
};

constexpr bool i2s_mode_is_host(I2sMode m) {
    return m == I2sMode::host_transmit || m == I2sMode::host_receive;
}
constexpr bool i2s_mode_is_transmit(I2sMode m) {
    return m == I2sMode::host_transmit || m == I2sMode::client_transmit;
}

/// I2SSTD[1:0] (20.3.2): the four audio standards - Philips (WS one
/// clock ahead of the MSB), MSB-justified (WS with the MSB), LSB-justified
/// (the datum's last bit against the channel's end) and PCM, whose frame
/// PCMSYNC makes short (a one-bit WS) or long (thirteen bits).
enum class I2sStandard : uint8_t {
    philips = 0,
    msb_justified = 1,
    lsb_justified = 2,
    pcm = 3,
};

/// DATLEN[1:0]: the datum a channel carries. The fourth code is "not
/// allowed" (20.4.8) and has no name here.
enum class I2sDataLength : uint8_t { bits16 = 0, bits24 = 1, bits32 = 2 };

/// CHLEN: the channel's width. "Only when DATLEN = 00, the write operation
/// of this bit is meaningful, otherwise the channel length is fixed to 32
/// bits by hardware" - which is why a wider datum in a 16-bit channel is
/// REFUSED here rather than quietly given the channel it would get.
enum class I2sChannelLength : uint8_t { bits16 = 0, bits32 = 1 };

struct I2sConfig {
    I2sMode mode = I2sMode::host_transmit;
    I2sStandard standard = I2sStandard::philips;
    bool pcm_long_frame = false;       ///< PCMSYNC, and only under `pcm`
    I2sDataLength data = I2sDataLength::bits16;
    I2sChannelLength channel = I2sChannelLength::bits16;
    bool clock_idle_high = false;      ///< CKPOL: the level CK rests at
    bool master_clock_out = false;     ///< MCKOE: MCK = 256 x FS on its own pad, a host's
    uint8_t div = 2;                   ///< I2SDIV, 2..255 (0 and 1 forbidden), a host's
    bool odd = false;                  ///< ODD: the divider is 2 x div + 1
    bool dma_transmit = false;
    bool dma_receive = false;
};

/// The channel width the silicon really uses: 32 bits whenever the datum
/// is wider than 16 (20.4.8) - which a VALID configuration states itself.
constexpr bool i2s_channel_is_32(const I2sConfig& c) {
    return c.data != I2sDataLength::bits16 || c.channel == I2sChannelLength::bits32;
}

/// Data-register accesses per channel (20.3.2): ONE for a 16-bit datum,
/// in a 16- or a 32-bit channel, and TWO for a 24- or 32-bit one - the
/// high half first - which is also two DMA items.
constexpr uint8_t i2s_accesses_per_channel(const I2sConfig& c) {
    return c.data == I2sDataLength::bits16 ? 1u : 2u;
}

/// The whole divisor 20.3.3 puts between I2SxCLK and one audio frame: two
/// channels of 16 or 32 bits times the linear divider 2 x I2SDIV + ODD -
/// or, with MCK out, a fixed 256 (16 x 2 x 8 and 32 x 2 x 4 alike).
constexpr uint32_t i2s_frame_factor(bool channel_32, bool master_clock_out) {
    if (master_clock_out) {
        return 256u;
    }
    return channel_32 ? 64u : 32u;
}

/// The sampling frequency a divider pair produces from an I2S clock; 0 for
/// a forbidden divider (I2SDIV of 0 or 1).
constexpr uint32_t i2s_fs_hz(uint32_t i2s_clk_hz, uint8_t div, bool odd, bool channel_32,
                             bool master_clock_out) {
    if (div < 2u) {
        return 0u;
    }
    const uint32_t whole = 2u * static_cast<uint32_t>(div) + (odd ? 1u : 0u);
    const uint32_t den = i2s_frame_factor(channel_32, master_clock_out) * whole;
    return den == 0u ? 0u : i2s_clk_hz / den;
}

/// The same for a configuration, and the bit clock it puts on CK: two
/// channels of its width per frame.
constexpr uint32_t i2s_fs_hz(uint32_t i2s_clk_hz, const I2sConfig& c) {
    return i2s_fs_hz(i2s_clk_hz, c.div, c.odd, i2s_channel_is_32(c), c.master_clock_out);
}
constexpr uint32_t i2s_bit_clock_hz(uint32_t i2s_clk_hz, const I2sConfig& c) {
    return i2s_fs_hz(i2s_clk_hz, c) * (i2s_channel_is_32(c) ? 64u : 32u);
}

struct I2sPrescaler {
    uint8_t div = 0;
    bool odd = false;
};

/// The divider pair whose FS is nearest `fs_hz`, or nullopt when the
/// whole divisor would fall outside 4..511 (I2SDIV 2..255 with ODD). An
/// audio rate is hardly ever exact against a system clock, so the rounding
/// is to nearest and i2s_fs_hz() says what really comes out.
constexpr std::optional<I2sPrescaler> i2s_prescaler_for(uint32_t i2s_clk_hz, uint32_t fs_hz,
                                                        bool channel_32, bool master_clock_out) {
    if (i2s_clk_hz == 0u || fs_hz == 0u) {
        return {};
    }
    const uint32_t unit = i2s_frame_factor(channel_32, master_clock_out) * fs_hz;
    if (unit == 0u) {
        return {};
    }
    const uint32_t whole = (i2s_clk_hz + unit / 2u) / unit;
    if (whole < 4u || whole > 511u) {
        return {};
    }
    return I2sPrescaler{static_cast<uint8_t>(whole / 2u), (whole & 1u) != 0u};
}

/// The refusals the chapter owes: a field holding a value its register
/// field does not encode (DATLEN's fourth code among them); a host's
/// forbidden divider (I2SDIV 0 and 1, 20.4.9); PCM's long frame named
/// outside PCM; a datum wider than 16 bits asked of a 16-bit channel,
/// which is not a frame this block makes (20.4.8); and the master clock
/// asked of a client, whose MCKOE "is only used in I2S master mode".
constexpr bool i2s_config_valid(const I2sConfig& c) {
    if (static_cast<uint8_t>(c.mode) > 3u || static_cast<uint8_t>(c.standard) > 3u ||
        static_cast<uint8_t>(c.data) > 2u || static_cast<uint8_t>(c.channel) > 1u) {
        return false;
    }
    if (i2s_mode_is_host(c.mode) && c.div < 2u) {
        return false;
    }
    if (c.pcm_long_frame && c.standard != I2sStandard::pcm) {
        return false;
    }
    if (c.data != I2sDataLength::bits16 && c.channel == I2sChannelLength::bits16) {
        return false;
    }
    if (c.master_clock_out && !i2s_mode_is_host(c.mode)) {
        return false;
    }
    return true;
}

/// SPIx_I2S_CFGR for a configuration, I2SE clear.
constexpr uint16_t i2s_i2scfgr_of(const I2sConfig& c) {
    uint16_t v = spi_i2smod;
    v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.mode) << spi_i2s_cfg_shift));
    v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.standard) << spi_i2s_std_shift));
    if (c.pcm_long_frame) { v = static_cast<uint16_t>(v | spi_i2s_pcmsync); }
    v = static_cast<uint16_t>(v | (static_cast<uint16_t>(c.data) << spi_i2s_datlen_shift));
    if (c.channel == I2sChannelLength::bits32) { v = static_cast<uint16_t>(v | spi_i2s_chlen); }
    if (c.clock_idle_high) { v = static_cast<uint16_t>(v | spi_i2s_ckpol); }
    return v;
}

/// SPIx_I2SPR for a configuration.
constexpr uint16_t i2s_i2spr_of(const I2sConfig& c) {
    uint16_t v = c.div;
    if (c.odd) { v = static_cast<uint16_t>(v | spi_i2s_odd); }
    if (c.master_clock_out) { v = static_cast<uint16_t>(v | spi_i2s_mckoe); }
    return v;
}

/**
 * 20.3.4.3's and 20.3.5.2's way to stop a RECEIVER without starting
 * another transfer: which RXNE to wait for - the LAST one, or the one
 * before it - and how many I2S clock periods to spend after it before
 * I2SE is cleared. A 16-bit datum in a 32-bit channel waits 17 periods
 * after the second-to-last RXNE when LSB-justified and one after the last
 * otherwise; every other host combination waits one after the
 * second-to-last; a client clears I2SE at the last RXNE.
 */
struct I2sReceiveStop {
    bool after_last = false;
    uint8_t clock_periods = 0;
};

constexpr I2sReceiveStop i2s_receive_stop(const I2sConfig& c) {
    if (!i2s_mode_is_host(c.mode)) {
        return {true, 0};
    }
    if (c.data == I2sDataLength::bits16 && c.channel == I2sChannelLength::bits32) {
        return c.standard == I2sStandard::lsb_justified ? I2sReceiveStop{false, 17}
                                                         : I2sReceiveStop{true, 1};
    }
    return {false, 1};
}

/// Whether THIS PART has the I2S face of instance `n`: the datasheet's
/// I2S row (device::has_i2s) and one of the two instances that carry it.
constexpr bool i2s_present(uint8_t n) {
    return device::has_i2s && (n == 2u || n == 3u) && spi_present(n);
}

/**
 * The I2S clock this class feeds a master: SYSCLK itself (figure 3-3 - the
 * simple tree - and RCC_CFGR2's I2S2SRC and I2S3SRC, whose other choice is
 * a PLL3 the CH32V30x_D8C has and this class has not). A static Clock's
 * sysclk_hz; under a DynamicClock the program hands the rate in force to
 * i2s_prescaler_for() itself.
 */
template <typename C>
constexpr uint32_t i2s_clock_hz(C clock) {
    static_assert(C::is_static,
                  "brio I2s: the I2S clock is SYSCLK, which a static Clock states - under a "
                  "DynamicClock the rate in force is the program's to hand to i2s_prescaler_for()");
    (void)clock;
    return C::sysclk_hz;
}

/// The four pads an I2S link can claim, as a COLUMN of the instance (the
/// datasheet's table 3-4, afio.hpp): the word select, the bit clock, the
/// data line and the optional master clock. They are the SPI's pads under
/// other names - WS is NSS's, CK is SCK's, SD is MOSI's - and I2S3's move
/// with SPI3's remap code; MCK sits on a pad of its own (PC6 for I2S2, PC7
/// for I2S3) that no remap moves.
struct I2sPins {
    Pad ws{};
    Pad ck{};
    Pad sd{};
    Pad mck{};
    uint8_t remap = 0;
};

/// The column of instance `n` under `code`, MCK named or not.
constexpr I2sPins i2s_pins_for(uint8_t n, uint8_t code = 0, bool with_mck = false) {
    const I2sPadSet p = n == 3 ? afio_i2s3_pads(code) : afio_i2s2_pad_set;
    return I2sPins{.ws = p.ws, .ck = p.ck, .sd = p.sd, .mck = with_mck ? p.mck : Pad{},
                   .remap = static_cast<uint8_t>(n == 3 ? code : 0u)};
}

template <uint8_t n>
inline constexpr I2sPins i2s_default_pins = i2s_pins_for(n, 0);

/// Is that set a column of instance `n` on this part? WS, CK and SD are
/// required and must be the column's own pads, the code must be a column
/// the instance has, MCK is optional and must be the instance's OWN master
/// clock pad, and every pad named must be one the package bonds.
constexpr bool i2s_pins_valid(uint8_t n, const I2sPins& p) {
    if ((n != 2u && n != 3u) || p.remap >= (n == 3u ? afio_spi3_codes : 1u)) {
        return false;
    }
    const I2sPins column = i2s_pins_for(n, p.remap, true);
    if (!(p.ws == column.ws && p.ck == column.ck && p.sd == column.sd)) {
        return false;
    }
    if (p.mck.valid() && !(p.mck == column.mck)) {
        return false;
    }
    return pad_bonded(p.ws) && pad_bonded(p.ck) && pad_bonded(p.sd) &&
           (!p.mck.valid() || pad_bonded(p.mck));
}

/**
 * I2s<n> - SPI2's or SPI3's register block in its audio face, a RESOURCE
 * and nothing above it (the file header).
 *
 *   using Out = brio::I2s<2>;
 *   Out::bus_clock(true);
 *   Out::reset();
 *   Out::claim_pads<brio::i2s_default_pins<2>>(brio::I2sMode::host_transmit);
 *   const auto pre = *brio::i2s_prescaler_for(brio::i2s_clock_hz(clock), 48'000, false, false);
 *   (void)Out::configure({.mode = brio::I2sMode::host_transmit, .div = pre.div, .odd = pre.odd});
 *   Out::enable();
 *
 * WHAT IS SHARED WITH THE SPI FACE AND WHAT IS NOT. The gate, the reset,
 * the vector, DATAR, STATR and CTLR2's interrupt and DMA enables are one
 * block's and are reached here by the same names; CTLR1, the CRC
 * registers, SSOE, MODF and CRCERR are not used in I2S mode (20.3.1), and
 * I2SMOD is what decides which face answers. `configure()` writes I2SPR,
 * then I2S_CFGR whole, then CTLR2's two DMA bits - 20.3.4.1's order -
 * with I2SE and SPE clear on the way in, because I2SMOD "can only be set
 * when SPI or I2S is disabled" and every other field of both registers
 * "should be set when I2S is turned off".
 */
template <uint8_t n>
struct I2s {
    static_assert(device::has_i2s,
                  "brio I2s: this part has no I2S - the datasheets give two to the CH32V303RC and "
                  "VC alone (table 2-1-1's I2S row; the CH32V203's table 2-1 has none)");
    static_assert(n == 2u || n == 3u,
                  "brio I2s: the I2S face is SPI2's and SPI3's (20.3.1, tables 20-2 and 20-3)");

    I2s() = delete;

    using Block = Spi<n>;

    static constexpr uint8_t number = n;
    static constexpr Irq irq = Block::irq;
    /// The SPI face's two requests, the same two slots (20.3.9).
    static constexpr DmaSlot dma_rx_slot = Block::dma_rx_slot;
    static constexpr DmaSlot dma_tx_slot = Block::dma_tx_slot;

    static SpiRegs& regs() { return Block::regs(); }
    static volatile void* data_address() { return Block::data_address(); }

    // The gate, the reset and the column are the instance's.
    static void bus_clock(bool on) { Block::bus_clock(on); }
    static void reset() { Block::reset(); }
    static bool remap(uint8_t code) { return Block::remap(code); }

    /// The whole configuration (the class comment's order), refused by
    /// i2s_config_valid()'s rules with nothing written.
    static bool configure(const I2sConfig& c) {
        if (!i2s_config_valid(c)) {
            return false;
        }
        write_config(c);
        return true;
    }

    /// The same where the configuration is a CONSTANT: the refusal is a
    /// compile error on the line that wrote it.
    template <I2sConfig c>
    static void configure() {
        static_assert(i2s_config_valid(c),
                      "brio I2s: this configuration is not one chapter 20 has - a field holds a "
                      "value its register field does not encode, a host's divider is 0 or 1, PCM's "
                      "long frame is named outside PCM, a datum wider than 16 bits is asked of a "
                      "16-bit channel (20.4.8: that channel is 32 bits whatever CHLEN says), or "
                      "the master clock is asked of a client");
        write_config(c);
    }

    /// Back to the SPI face: I2SE and I2SMOD cleared together.
    static void select_spi_mode() {
        regs().I2SCFGR = static_cast<uint16_t>(regs().I2SCFGR & ~(spi_i2se | spi_i2smod));
    }
    static bool i2s_mode_selected() { return (regs().I2SCFGR & spi_i2smod) != 0u; }

    static void enable() { regs().I2SCFGR = static_cast<uint16_t>(regs().I2SCFGR | spi_i2se); }
    static bool enabled() { return (regs().I2SCFGR & spi_i2se) != 0u; }

    /// A TRANSMITTER stops after its last frame is out - TXE, then BSY
    /// down, 20.3.4.2's advice - each wait bounded, false when one ran
    /// out and I2SE was cleared regardless. A RECEIVER stops where it
    /// stands: the way to end a reception on a frame's edge is
    /// i2s_receive_stop()'s, which is timing the caller owns.
    static bool disable() {
        bool ok = true;
        if (enabled() && i2s_mode_is_transmit(mode())) {
            ok = wait_until([] { return tx_empty(); }) && wait_until([] { return !busy(); });
        }
        regs().I2SCFGR = static_cast<uint16_t>(regs().I2SCFGR & ~spi_i2se);
        return ok;
    }

    static I2sMode mode() {
        return static_cast<I2sMode>((regs().I2SCFGR & spi_i2s_cfg_mask) >> spi_i2s_cfg_shift);
    }
    static I2sStandard standard() {
        return static_cast<I2sStandard>((regs().I2SCFGR & spi_i2s_std_mask) >> spi_i2s_std_shift);
    }

    // ---- the generator -------------------------------------------------------

    /// I2SDIV and ODD alone, with the block disabled: what a program that
    /// changes the sampling rate and nothing else writes. MCKOE is kept.
    static bool prescaler(uint8_t div, bool odd) {
        if (div < 2u || enabled()) {
            return false;
        }
        regs().I2SPR = static_cast<uint16_t>((regs().I2SPR & spi_i2s_mckoe) | div |
                                             (odd ? spi_i2s_odd : uint16_t{0}));
        return true;
    }
    static uint8_t prescaler_div() { return static_cast<uint8_t>(regs().I2SPR & spi_i2s_div_mask); }
    static bool prescaler_odd() { return (regs().I2SPR & spi_i2s_odd) != 0u; }
    static bool master_clock_out() { return (regs().I2SPR & spi_i2s_mckoe) != 0u; }

    // ---- data and flags ------------------------------------------------------

    static void data(uint16_t v) { regs().DATAR = v; }
    static uint16_t data() { return regs().DATAR; }

    static bool tx_empty() { return (regs().STATR & spi_txe) != 0u; }
    static bool rx_ready() { return (regs().STATR & spi_rxne) != 0u; }
    /// BSY; 20.3.6.1: always LOW in master receive, whatever is moving.
    static bool busy() { return (regs().STATR & spi_bsy) != 0u; }
    static uint16_t status() { return regs().STATR; }
    /// CHSIDE, refreshed when TXE rises on a transmitter and when a datum
    /// lands on a receiver: false the left channel, true the right. No
    /// meaning under PCM, nor after an underrun or an overrun (20.3.6.4).
    static bool right_channel() { return (regs().STATR & spi_chside) != 0u; }

    /// UDR: a client transmitter clocked before software loaded DATAR
    /// (20.3.7.1), cleared by a read of STATR.
    static bool underrun() { return (regs().STATR & spi_udr) != 0u; }
    static void clear_underrun() { (void)regs().STATR; }
    /// OVR: DATAR then STATR, as on the SPI face (20.3.7.2).
    static bool overrun() { return (regs().STATR & spi_ovr) != 0u; }
    static void clear_overrun() {
        (void)regs().DATAR;
        (void)regs().STATR;
    }

    static void dma_requests(bool tx, bool rx) { Block::dma_requests(tx, rx); }
    static void rxne_interrupt(bool on) { Block::rxne_interrupt(on); }
    static void txe_interrupt(bool on) { Block::txe_interrupt(on); }
    static void error_interrupt(bool on) { Block::error_interrupt(on); }

    /// The raised-and-enabled sources: RXNE and TXE under their own
    /// enables, OVR and UDR under ERRIE (20.3.8). Nothing is cleared.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint16_t st = regs().STATR;
        const uint16_t en = regs().CTLR2;
        uint32_t up = 0;
        if ((en & spi_rxneie) != 0u && (st & spi_rxne) != 0u) { up |= SpiFlag::rxne; }
        if ((en & spi_txeie) != 0u && (st & spi_txe) != 0u) { up |= SpiFlag::txe; }
        if ((en & spi_errie) != 0u) {
            up |= st & SpiFlag::i2s_errors;
        }
        return up;
    }

    // ---- the pads (table 10-5) -----------------------------------------------

    /// Hand a column's pads to the face for a mode: WS and CK as push-pull
    /// alternate outputs on a host and floating inputs on a client, SD an
    /// output on a transmitter and a pulled-up input on a receiver, MCK an
    /// output where the column names it and the mode is a host's. The
    /// column's remap code is written here, SPI3's field moving the SPI
    /// face's pads with it.
    template <I2sPins pins>
    static bool claim_pads(I2sMode m) {
        static_assert(i2s_pins_valid(n, pins),
                      "brio I2s: these pads are not a column of this instance on this part - WS, "
                      "CK and SD must be the column's own (I2S2: PB12/PB13/PB15; I2S3: PA15/PB3/PB5 "
                      "or PA4/PC10/PC12), the master clock the instance's own pad (PC6 for I2S2, "
                      "PC7 for I2S3), and every pad one the package bonds");
        if (!remap(pins.remap)) {
            return false;
        }
        using Ws = Pin<pins.ws.port, pins.ws.pin>;
        using Ck = Pin<pins.ck.port, pins.ck.pin>;
        using Sd = Pin<pins.sd.port, pins.sd.pin>;
        if (i2s_mode_is_host(m)) {
            Ws::function();
            Ck::function();
        } else {
            Ws::input();
            Ck::input();
        }
        if (i2s_mode_is_transmit(m)) {
            Sd::function();
        } else {
            Sd::input(PinPull::up);
        }
        if constexpr (pins.mck.valid()) {
            using Mck = Pin<pins.mck.port, pins.mck.pin>;
            if (i2s_mode_is_host(m)) {
                Mck::function();
            } else {
                Mck::release();
            }
        }
        return true;
    }

    /// The column's pads back to floating inputs, driving nothing.
    template <I2sPins pins>
    static void release_pads() {
        static_assert(i2s_pins_valid(n, pins), "brio I2s: not a column of this instance");
        Pin<pins.ws.port, pins.ws.pin>::release();
        Pin<pins.ck.port, pins.ck.pin>::release();
        Pin<pins.sd.port, pins.sd.pin>::release();
        if constexpr (pins.mck.valid()) {
            Pin<pins.mck.port, pins.mck.pin>::release();
        }
    }

private:
    static void write_config(const I2sConfig& c) {
        SpiRegs& r = regs();
        r.I2SCFGR = static_cast<uint16_t>(r.I2SCFGR & ~spi_i2se);
        r.CTLR1 = static_cast<uint16_t>(r.CTLR1 & ~spi_spe);
        r.I2SPR = i2s_i2spr_of(c);
        r.I2SCFGR = i2s_i2scfgr_of(c);
        uint16_t v = static_cast<uint16_t>(r.CTLR2 & ~(spi_txdmaen | spi_rxdmaen | spi_ssoe));
        if (c.dma_transmit) { v = static_cast<uint16_t>(v | spi_txdmaen); }
        if (c.dma_receive) { v = static_cast<uint16_t>(v | spi_rxdmaen); }
        r.CTLR2 = v;
    }

    /// A bounded wait, the SPI face's.
    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = 400'000u; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }
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
static_assert(spi_base_for(1) == 0x40013000UL && spi_base_for(2) == 0x40003800UL &&
              spi_base_for(3) == 0x40003C00UL && spi_base_for(4) == 0);
static_assert(spi_bus_for(1) == Bus::pb2 && spi_bus_for(2) == Bus::pb1 && spi_bus_for(3) == Bus::pb1);
static_assert(spi_gate_for(1) == rcc_pb2_spi1 && spi_gate_for(2) == rcc_pb1_spi2 &&
              spi_gate_for(3) == rcc_pb1_spi3);
static_assert(spi_irq_for(1) == Irq::spi1 && spi_irq_for(2) == Irq::spi2 &&
              spi_irq_for(3) == Irq::spi3);
static_assert(spi_bus_hz_at<1>(144'000'000UL) == 144'000'000UL);
static_assert(spi_bus_hz_at<2>(144'000'000UL) == 72'000'000UL);
static_assert(spi_bus_hz_at<3>(144'000'000UL) == 72'000'000UL);

// Table 11-5's four rows, through dma_engine.hpp - and SPI3's two on DMA2
// where the part has the instance (table 11-3).
static_assert(spi_dma_rx_channel(1) == 2 && spi_dma_tx_channel(1) == 3);
static_assert(spi_present(2) == (device::spi_count >= 2u));
static_assert(spi_present(3) == (device::spi_count >= 3u));
static_assert(!spi_present(3) ||
              (spi_dma_rx_slot(3) == DmaSlot{2, 1} && spi_dma_tx_slot(3) == DmaSlot{2, 2}));

// Tables 10-32 and 10-33: SPI1's two columns, SPI2's single one, SPI3's
// two - the first of which is SPI1's second.
static_assert(spi_column_count(1) == 2 && spi_column_count(2) == 1 && spi_column_count(3) == 2);
static_assert(spi_pins_for(3, 0).nss == Pad{'A', 15} && spi_pins_for(3, 0).sck == Pad{'B', 3} &&
              spi_pins_for(3, 0).miso == Pad{'B', 4} && spi_pins_for(3, 0).mosi == Pad{'B', 5});
static_assert(spi_pins_for(3, 1).nss == Pad{'A', 4} && spi_pins_for(3, 1).sck == Pad{'C', 10} &&
              spi_pins_for(3, 1).remap == 1u);
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

// 20.3.3's arithmetic: FS is I2SxCLK over 32 or 64 bits a frame times the
// linear divider, or over 256 of it with MCK out - at SYSCLK = 144 MHz a
// 48 kHz frame of two 16-bit channels is a divider of 94 and comes out at
// 47872 Hz, and the ladder's two ends are refused rather than rounded.
static_assert(i2s_frame_factor(false, false) == 32u && i2s_frame_factor(true, false) == 64u);
static_assert(i2s_frame_factor(false, true) == 256u && i2s_frame_factor(true, true) == 256u);
static_assert(i2s_prescaler_for(144'000'000UL, 48'000UL, false, false)->div == 47u &&
              !i2s_prescaler_for(144'000'000UL, 48'000UL, false, false)->odd);
static_assert(i2s_fs_hz(144'000'000UL, 47, false, false, false) == 47'872UL);
static_assert(i2s_prescaler_for(144'000'000UL, 48'000UL, true, false)->div == 23u &&
              i2s_prescaler_for(144'000'000UL, 48'000UL, true, false)->odd);
static_assert(i2s_fs_hz(144'000'000UL, 23, true, true, false) == 47'872UL);
static_assert(i2s_fs_hz(144'000'000UL, 1, false, false, false) == 0u);
static_assert(i2s_prescaler_for(144'000'000UL, 8'000UL, false, true)->div == 35u);
static_assert(!i2s_prescaler_for(144'000'000UL, 1'000UL, false, false).has_value());
static_assert(!i2s_prescaler_for(8'000'000UL, 96'000UL, true, true).has_value());
static_assert(i2s_bit_clock_hz(144'000'000UL, I2sConfig{.div = 47}) == 47'872UL * 32u);

// The refusals i2s_config_valid() owes, and the two register words.
static_assert(i2s_config_valid(I2sConfig{}));
static_assert(!i2s_config_valid(I2sConfig{.div = 1}));
static_assert(i2s_config_valid(I2sConfig{.mode = I2sMode::client_receive, .div = 0}));
static_assert(!i2s_config_valid(I2sConfig{.pcm_long_frame = true}));
static_assert(i2s_config_valid(I2sConfig{.standard = I2sStandard::pcm, .pcm_long_frame = true}));
static_assert(!i2s_config_valid(I2sConfig{.data = I2sDataLength::bits24}));
static_assert(i2s_config_valid(I2sConfig{.data = I2sDataLength::bits24,
                                         .channel = I2sChannelLength::bits32}));
static_assert(!i2s_config_valid(I2sConfig{.data = static_cast<I2sDataLength>(3),
                                          .channel = I2sChannelLength::bits32}));
static_assert(!i2s_config_valid(I2sConfig{.mode = I2sMode::client_transmit,
                                          .master_clock_out = true}));
static_assert(i2s_i2scfgr_of(I2sConfig{.mode = I2sMode::host_receive,
                                       .standard = I2sStandard::pcm, .pcm_long_frame = true,
                                       .data = I2sDataLength::bits32,
                                       .channel = I2sChannelLength::bits32,
                                       .clock_idle_high = true}) ==
              (spi_i2smod | (3u << spi_i2s_cfg_shift) | (3u << spi_i2s_std_shift) |
               spi_i2s_pcmsync | (2u << spi_i2s_datlen_shift) | spi_i2s_chlen | spi_i2s_ckpol));
static_assert(i2s_i2spr_of(I2sConfig{.master_clock_out = true, .div = 3, .odd = true}) ==
              (3u | spi_i2s_odd | spi_i2s_mckoe));
static_assert(i2s_accesses_per_channel(I2sConfig{}) == 1u &&
              i2s_accesses_per_channel(I2sConfig{.data = I2sDataLength::bits24,
                                                 .channel = I2sChannelLength::bits32}) == 2u);
// 20.3.4.3's stops.
static_assert(i2s_receive_stop(I2sConfig{.mode = I2sMode::host_receive,
                                         .standard = I2sStandard::lsb_justified,
                                         .channel = I2sChannelLength::bits32}).clock_periods == 17u);
static_assert(i2s_receive_stop(I2sConfig{.mode = I2sMode::host_receive,
                                         .channel = I2sChannelLength::bits32}).after_last);
static_assert(!i2s_receive_stop(I2sConfig{.mode = I2sMode::host_receive}).after_last);
static_assert(i2s_receive_stop(I2sConfig{.mode = I2sMode::client_receive}).after_last);

// The columns: I2S2 on SPI2's pads with MCK on PC6, I2S3 on SPI3's with MCK
// on PC7 in either column; an MCK that is not the instance's is refused.
static_assert(i2s_pins_for(2).ws == Pad{'B', 12} && i2s_pins_for(2).ck == Pad{'B', 13} &&
              i2s_pins_for(2).sd == Pad{'B', 15} && !i2s_pins_for(2).mck.valid() &&
              i2s_pins_for(2, 0, true).mck == Pad{'C', 6});
static_assert(i2s_pins_for(3).ws == Pad{'A', 15} && i2s_pins_for(3, 1).ck == Pad{'C', 10} &&
              i2s_pins_for(3, 1, true).mck == Pad{'C', 7} && i2s_pins_for(3, 1).remap == 1u);
static_assert(!i2s_pins_valid(2, I2sPins{.ws = {'B', 12}, .ck = {'B', 13}, .sd = {'B', 15},
                                         .mck = {'C', 7}}));
static_assert(!i2s_pins_valid(1, i2s_pins_for(2)));
static_assert(!i2s_pins_valid(2, I2sPins{.ws = {'B', 12}, .ck = {'B', 13}, .sd = {'B', 14}}));
static_assert(i2s_present(2) == device::has_i2s && i2s_present(3) == device::has_i2s);

}  // namespace brio
