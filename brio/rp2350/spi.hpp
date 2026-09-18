/*
 * spi.hpp
 *
 * The SPI (datasheet 12.3: two ARM PrimeCell SSPs, PL022 revision r1p4).
 * The chapter's driver is NOT here: the PL022 is a licensed design this
 * chip shares with others, so the resource, the host engine and the
 * client are written once in the IP stratum (brio/pl022/spi.hpp,
 * docs/pl022/README.md) and this file is what that file asks of a
 * family -
 *
 *  - the pin table of 9.4 (table 645) with its function code;
 *  - `Rp2350Pl022`, the CHIP TRAITS: the register block and where each
 *    instance's sits, the reset bits of 7.5, the interrupt lines of 3.2,
 *    the atomic set/clear aliases of 2.1.3, the DREQ numbers of
 *    12.6.4.1, the pad type with the setup each signal wants, the
 *    run-time pin a Request carries as its select, the microsecond
 *    busy-wait `cs_setup_us` is spent on, and WHICH RATE IS SSPCLK -
 *    clk_peri (12.3, and the note under 12.3.3.3), which
 *    rp2350/clock.hpp feeds from clk_sys undivided by default, so the
 *    bit rate comes from Clock::pclk_hz and never from a second
 *    statement of the rate;
 *  - the PUBLIC NAMES: `Pl022<n>` the resource, `SpiHost<n, pins, ...>`
 *    the engine `SpiBus` drives and `SpiClient<n, pins>` the other end,
 *    this chip's aliases of the three IP templates.
 *
 * THE PINS, AND THE COLUMN THIS CHAPTER DID NOT GAIN. The pads march in
 * groups of four - RX, CSn, SCK, TX - and the groups cycle SPI0, SPI0,
 * SPI1, SPI1 and again, so GP0..GP7 are SPI0's, GP8..GP15 SPI1's,
 * GP16..GP23 SPI0's, all the way to GP47 on the QFN-80: the instance is
 * bit 3 of the pad number. That is the RP2040's rule over a longer bank,
 * and UNLIKE THE UART THIS CHAPTER HAS NO SECOND FUNCTION COLUMN - the
 * table's F11 entries are the UART's alternate data pads and carry no
 * SPI signal on any pad. So `pad_function` here is one constant,
 * `PinFunction::spi` (F1), and a pin set is four plain pad numbers.
 *
 * WHAT THIS SILICON PUTS AROUND THE BLOCK. Two FIFOs of eight frames
 * each (12.3.3.4, 12.3.3.5), so a host keeps up to eight frames in
 * flight. The bit rate is clk_peri / (CPSDVSR x (1 + SCR)) with the
 * prescaler even in 2..254 and SCR in 0..255, which at the top of a 150
 * MHz clk_peri is 75 Mbit/s for a host; a CLIENT needs SSPCLK at least
 * twelve times its own clock (12.3.4.4), so 12.5 Mbit/s here. The pads
 * come up ISOLATED and pulled down (9.11): every configuring verb of
 * pin.hpp drops the latch as it writes, and a host's receive pad is
 * handed over with a pull-UP so an answer line nothing drives reads as
 * the idle ones a bus expects.
 *
 * WHAT 12.3.1 SAYS, AND WHAT THE PADS DO. The chapter says two different
 * things about the block's pad-enable output nSSPOE: 12.3.1 ("Changes
 * from RP2040") has it control the output enable of the SSPTXD pad, the
 * peripheral tristating its output when deselected in client mode; the
 * idle-level lists of 12.3.4.10 to 12.3.4.14 say, five times in the same
 * words, that the signal "is not connected to the pad in RP2350". THE
 * PADS ANSWER THE SECOND (measured, on stepping A2, by an instrument
 * proven both ways in the same letter): a deselected client and a client
 * with SOD set both leave the transmit pad DRIVEN, as on the RP2040. What
 * SOD does do is keep the answers off the wire - the host clocks a steady
 * 0xFF with frames waiting in the client's FIFO. The driver depends on
 * neither reading: `Pl022Client::drive_output(false)` makes the dark
 * listener by releasing the PAD, and `sod()` stays a bit of the resource.
 *
 * THE LINE. The four maskable sources of an instance combine into ONE
 * interrupt line (SPI0_IRQ = 31, SPI1_IRQ = 32, datasheet 3.2); the app
 * binds isr_spi0 / isr_spi1 to the instance's isr() body, and those names
 * are bound on BOTH architectures - a Cortex-M vector table on one half,
 * Hazard3's own dispatch on the other - because the interrupt numbering
 * is shared (3.8.4.2). Which is also why `Interrupts` below is the
 * controller core.hpp names and never an NVIC.
 *
 * THE DMA ENGINES ride any two channels, told the instance's requests
 * (Dreq::spi{n}_tx / spi{n}_rx) at arm(). THOSE ROWS ARE NOT THE
 * RP2040'S: a third PIO moved every number above the first PIO's, so the
 * SPIs sit at 24..27 where they sat at 16..19 (rp2350/dma_engine.hpp).
 * The IP file raises the requests around the engines and nowhere else,
 * which is what this chip's controller wants.
 */

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"

// This chip's busy-wait comes FIRST: the traits below hand the IP file
// `DelayRate`, a type of this family, and the driver spends `cs_setup_us`
// by calling that type's own verb - so the verb is in scope where the
// driver that calls it is defined.
#include "rp2350/delay.hpp"

#include "pl022/spi.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/core.hpp"
#include "rp2350/dma_engine.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/resets.hpp"

namespace brio {

// ---- the pads --------------------------------------------------------------------

/// Which pins carry the four signals: each the instance's own under
/// function 1, or none (0xFF). The host's `cs` is NOT the peripheral's
/// (a GPIO the Request carries); the client's IS.
struct SpiPins {
    uint8_t sck = 0xFFu;
    uint8_t tx = 0xFFu;
    uint8_t rx = 0xFFu;
    uint8_t cs = 0xFFu;
};

/// Which instance the four pads of `pin`'s group belong to (9.4): the
/// groups cycle SPI0, SPI0, SPI1, SPI1 over the whole bank, so the
/// instance is bit 3 of the pad number.
constexpr uint8_t spi_pad_instance(uint8_t pin) { return static_cast<uint8_t>((pin >> 3) & 1u); }

/// The four signals of a group, in the table's order: RX, CSn, SCK, TX.
///
/// The PACKAGE is not asked here: every pin's registers exist on both
/// packages and only the bond wire does not, so a pad the QFN-60 has not
/// got is refused one level down, by `Pin<n>`'s own static_assert, which
/// says so in those words.
constexpr bool spi_rx_pin(uint8_t n, uint8_t p) {
    return p < gpio_count_max && (p & 3u) == 0u && spi_pad_instance(p) == n;
}
constexpr bool spi_cs_pin(uint8_t n, uint8_t p) {
    return p < gpio_count_max && (p & 3u) == 1u && spi_pad_instance(p) == n;
}
constexpr bool spi_sck_pin(uint8_t n, uint8_t p) {
    return p < gpio_count_max && (p & 3u) == 2u && spi_pad_instance(p) == n;
}
constexpr bool spi_tx_pin(uint8_t n, uint8_t p) {
    return p < gpio_count_max && (p & 3u) == 3u && spi_pad_instance(p) == n;
}

/// A link needs SCK; every pin named must be the instance's own for
/// its signal; a signal the set leaves out is 0xFF.
constexpr bool spi_pins_valid(uint8_t n, const SpiPins& p) {
    if (n > 1u || !spi_sck_pin(n, p.sck)) {
        return false;
    }
    if (p.tx != 0xFFu && !spi_tx_pin(n, p.tx)) { return false; }
    if (p.rx != 0xFFu && !spi_rx_pin(n, p.rx)) { return false; }
    if (p.cs != 0xFFu && !spi_cs_pin(n, p.cs)) { return false; }
    return true;
}

// ---- what this chip tells the PL022 driver ----------------------------------------

/**
 * The chip traits brio/pl022/spi.hpp is instantiated with: every fact
 * about the PL022 that is the RP2350's rather than ARM's, and nothing
 * else. Nothing above this file ever names it.
 */
struct Rp2350Pl022 {
    Rp2350Pl022() = delete;

    using Regs = SPI0_Type;
    /// What the controller calls a line. `Interrupts` below is the
    /// controller itself, which on this chip is `brio::Irq` - an NVIC
    /// under one architecture and Hazard3's own under the other, named
    /// once in core.hpp. The qualified spelling is what keeps this
    /// member from shadowing it.
    using Irq = IRQn_Type;
    using Interrupts = ::brio::Irq;
    using Pins = SpiPins;
    using PinRef = brio::PinRef;
    using DmaRequest = Dreq;
    using DelayRate = brio::DelayRate;

    /// SPI0 and SPI1, both with 8-deep FIFOs (12.3.3.4, 12.3.3.5 - the
    /// same synthesis as the RP2040's).
    static constexpr uint8_t instances = 2;
    static constexpr uint8_t fifo_depth = 8;
    /// What a pin set puts in a signal it does not route.
    static constexpr uint8_t no_pad = 0xFFu;

    template <uint8_t i>
    static Regs& regs() { return *(i == 0 ? SPI0 : SPI1); }
    template <uint8_t i>
    static constexpr Irq irq() { return i == 0 ? SPI0_IRQ_IRQn : SPI1_IRQ_IRQn; }

    /// The instance's bit in the subsystem reset controller (7.5) - bits
    /// 18 and 19 here, where the RP2040 had 16 and 17.
    template <uint8_t i>
    static constexpr uint32_t reset_bit() { return i == 0 ? ResetBlock::spi0 : ResetBlock::spi1; }

    template <uint8_t i>
    static bool reset() { return Resets::cycle(reset_bit<i>()); }
    template <uint8_t i>
    static bool released() { return Resets::released(reset_bit<i>()); }
    template <uint8_t i>
    static void hold() { Resets::hold(reset_bit<i>()); }

    /// 12.6.4.1: the two data requests of an instance. NOT the RP2040's
    /// numbers - a third PIO moved every row above it (dma_engine.hpp).
    template <uint8_t i>
    static constexpr DmaRequest tx_request() { return i == 0 ? Dreq::spi0_tx : Dreq::spi1_tx; }
    template <uint8_t i>
    static constexpr DmaRequest rx_request() { return i == 0 ? Dreq::spi0_rx : Dreq::spi1_rx; }

    /// The atomic register aliases of 2.1.3: one bus write, no
    /// read-modify-write.
    static void set_bits(volatile uint32_t& reg, uint32_t bits) { hw_set(reg, bits); }
    static void clear_bits(volatile uint32_t& reg, uint32_t bits) { hw_clear(reg, bits); }

    /// The pin table above, and the four signals of a pin set.
    static constexpr bool pins_valid(uint8_t n, const Pins& p) { return spi_pins_valid(n, p); }
    static constexpr uint8_t sck_pad(const Pins& p) { return p.sck; }
    static constexpr uint8_t tx_pad(const Pins& p) { return p.tx; }
    static constexpr uint8_t rx_pad(const Pins& p) { return p.rx; }
    static constexpr uint8_t cs_pad(const Pins& p) { return p.cs; }

    /// The pad as a type, the function code that routes an SPI to it
    /// (9.4: F1 on every SPI pin of both packages, one column and no
    /// other), and the electrical setup each signal wants. Every
    /// configuring verb of pin.hpp drops the ISOLATION LATCH as it
    /// writes, which is what makes a pad of this chip answer at all.
    ///
    /// The two PULLED ones are the HOST's receive - this chip's pads come
    /// up pulled DOWN, so an answer line with nothing driving it would
    /// read as zeros rather than as the idle ones a bus expects - and the
    /// CLIENT's select, so an unwired select reads not-selected. The
    /// client's receive is driven by the host whenever it matters and
    /// takes no pull.
    template <uint8_t pin>
    using Pad = Pin<pin>;
    static constexpr PinFunction pad_function = PinFunction::spi;
    static constexpr PinConfig sck_pad_config() { return {}; }
    static constexpr PinConfig tx_pad_config() { return {}; }
    static constexpr PinConfig host_rx_pad_config() { return {.pull = PinPull::up}; }
    static constexpr PinConfig client_rx_pad_config() { return {}; }
    static constexpr PinConfig cs_pad_config() { return {.pull = PinPull::up}; }

    /// Two engines of one host must not share a channel: on this chip a
    /// request is a field any channel takes, so the CHANNEL is the
    /// identity (rp2350/dma_engine.hpp).
    template <typename Tx, typename Rx>
    static constexpr bool engines_distinct() { return uart_engines_distinct<Tx, Rx>(); }

    /// The microsecond busy-wait of this target (rp2350/delay.hpp): the
    /// ruler is the platform timer and not a core counter, so the factor
    /// carries no rate - only whether the caller had a clock at all. A
    /// `cs_setup_us` is served once `Mtime::start(clock)` has run, and
    /// refused (no time spent) before that.
    static constexpr DelayRate delay_rate(uint32_t hz) { return brio::delay_rate(hz); }

    /// SSPCLK is clk_peri (12.3) - `Clock::pclk_hz`, the one truth about
    /// the rate the bit rate is divided from.
    template <typename Clock>
    static constexpr uint32_t sspclk_hz(Clock) { return Clock::pclk_hz; }
};

static_assert(Pl022Chip<Rp2350Pl022>);

// ---- the public names --------------------------------------------------------------

/// The resource: which instance, where its registers are, its reset bit,
/// its interrupt line, and the whole of the PL022's programmer's model.
template <uint8_t n>
using Pl022 = Pl022Ssp<Rp2350Pl022, n>;

/**
 * The engine `SpiBus` (= `BusMaster`) drives, over Pl022<n>.
 *
 *   constexpr brio::SpiPins pins{.sck = 18, .tx = 19, .rx = 16};
 *   using Bus = brio::SpiHost<0, pins>;
 *   using Spi = brio::SpiBus<Bus, P, 4>;              // the arbiter AO
 *   using Cs = brio::Pin<17>;
 *
 *   Bus::init(clock, 8'000'000);                      // a ceiling for the bus
 *   Cs::output(true);
 *
 *   extern "C" void isr_spi0() {
 *       if (Bus::isr()) { brio::post<Spi>(brio::TransferDone{Bus::status()}); }
 *   }
 *
 * The same source builds for both architectures: `isr_spi0` is a
 * Cortex-M vector on one half and a Hazard3 dispatch entry on the other,
 * under one name.
 *
 * The two engine slots take rp2350/dma.hpp's DmaTxEngine / DmaRxEngine
 * on any two channels, both or neither.
 */
template <uint8_t n, SpiPins pins, typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
using SpiHost = Pl022Host<Rp2350Pl022, n, pins, TxEngine, RxEngine>;

/**
 * The other end of the wire, over Pl022<n>: the select goes to the
 * PERIPHERAL here (the PL022 in slave mode frames on SSPFSSIN).
 *
 *   constexpr brio::SpiPins client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 9};
 *   using Client = brio::SpiClient<1, client_pins>;
 *   Client::init(clock, {.mode = brio::SpiMode::mode1});
 */
template <uint8_t n, SpiPins pins>
using SpiClient = Pl022Client<Rp2350Pl022, n, pins>;

// ---- this chip's device description against the IP's own words ----------------------

// The IP file spells the block's bits itself, because it may not read a
// vendor header; here is where the two are held against each other.
static_assert(SpiControl0::data_size_lsb == SPI_SSPCR0_DSS_LSB &&
              SpiControl0::data_size_bits == SPI_SSPCR0_DSS_BITS &&
              SpiControl0::format_lsb == SPI_SSPCR0_FRF_LSB &&
              SpiControl0::cpol == SPI_SSPCR0_SPO_BITS && SpiControl0::cpha == SPI_SSPCR0_SPH_BITS &&
              SpiControl0::rate_lsb == SPI_SSPCR0_SCR_LSB &&
              SpiControl0::rate_bits == SPI_SSPCR0_SCR_BITS);
static_assert(SpiControl1::loopback == SPI_SSPCR1_LBM_BITS && SpiControl1::enable == SPI_SSPCR1_SSE_BITS &&
              SpiControl1::client == SPI_SSPCR1_MS_BITS &&
              SpiControl1::output_disable == SPI_SSPCR1_SOD_BITS);
static_assert(SpiDataField::prescaler == SPI_SSPCPSR_CPSDVSR_BITS &&
              SpiDataField::frame == SPI_SSPDR_DATA_BITS);
static_assert(SpiFlag::tx_empty == SPI_SSPSR_TFE_BITS && SpiFlag::tx_not_full == SPI_SSPSR_TNF_BITS &&
              SpiFlag::rx_not_empty == SPI_SSPSR_RNE_BITS && SpiFlag::rx_full == SPI_SSPSR_RFF_BITS &&
              SpiFlag::busy == SPI_SSPSR_BSY_BITS);
// One layout over IMSC, RIS and MIS - and the two sources ICR can clear.
static_assert(SpiInterrupt::overrun == SPI_SSPIMSC_RORIM_BITS &&
              SpiInterrupt::rx_timeout == SPI_SSPIMSC_RTIM_BITS &&
              SpiInterrupt::rx == SPI_SSPIMSC_RXIM_BITS && SpiInterrupt::tx == SPI_SSPIMSC_TXIM_BITS &&
              SpiInterrupt::all == SPI_SSPIMSC_BITS);
static_assert(SpiInterrupt::overrun == SPI_SSPRIS_RORRIS_BITS &&
              SpiInterrupt::rx_timeout == SPI_SSPRIS_RTRIS_BITS &&
              SpiInterrupt::rx == SPI_SSPRIS_RXRIS_BITS && SpiInterrupt::tx == SPI_SSPRIS_TXRIS_BITS);
static_assert(SpiInterrupt::overrun == SPI_SSPMIS_RORMIS_BITS &&
              SpiInterrupt::rx_timeout == SPI_SSPMIS_RTMIS_BITS &&
              SpiInterrupt::rx == SPI_SSPMIS_RXMIS_BITS && SpiInterrupt::tx == SPI_SSPMIS_TXMIS_BITS);
static_assert((SpiInterrupt::overrun | SpiInterrupt::rx_timeout) == SPI_SSPICR_BITS &&
              SpiInterrupt::overrun == SPI_SSPICR_RORIC_BITS &&
              SpiInterrupt::rx_timeout == SPI_SSPICR_RTIC_BITS);
static_assert(SpiDmaControl::tx == SPI_SSPDMACR_TXDMAE_BITS &&
              SpiDmaControl::rx == SPI_SSPDMACR_RXDMAE_BITS);

// Table 645's one column, and a pin set from the wrong instance.
static_assert(spi_rx_pin(0, 0) && spi_cs_pin(0, 1) && spi_sck_pin(0, 2) && spi_tx_pin(0, 3));
static_assert(spi_rx_pin(1, 8) && spi_cs_pin(1, 9) && spi_sck_pin(1, 10) && spi_tx_pin(1, 11));
static_assert(spi_sck_pin(0, 18) && spi_tx_pin(0, 19) && spi_rx_pin(0, 16) && spi_cs_pin(0, 17));
static_assert(spi_sck_pin(0, 38) && spi_tx_pin(1, 47) && spi_rx_pin(1, 44) && spi_cs_pin(0, 33));
static_assert(!spi_sck_pin(1, 18) && !spi_tx_pin(0, 11) && !spi_sck_pin(0, 3) && !spi_rx_pin(0, 48));
static_assert(spi_pins_valid(0, {.sck = 18, .tx = 19, .rx = 16}) &&
              spi_pins_valid(1, {.sck = 10, .tx = 11, .rx = 8, .cs = 9}));
static_assert(!spi_pins_valid(0, {.sck = 10, .tx = 19}) && !spi_pins_valid(2, {.sck = 2}));

} // namespace brio
