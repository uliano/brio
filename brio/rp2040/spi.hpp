/*
 * spi.hpp
 *
 * The SPI (datasheet 4.4: two ARM PL022s). The chapter's driver is NOT
 * here: the PL022 is a licensed design this chip shares with others, so
 * the resource, the host engine and the client are written once in the
 * IP stratum (brio/pl022/spi.hpp, docs/pl022/README.md) and this file is
 * what that file asks of a family -
 *
 *  - the pin table of 2.19.2 (table 279) with its function code;
 *  - `Rp2040Pl022`, the CHIP TRAITS: the register block and where each
 *    instance's sits, the reset bits of 2.14, the NVIC lines of 2.3.2,
 *    the atomic set/clear aliases of 2.1.2, the DREQ numbers of table
 *    119, the pad type with the function code that routes an SPI to it
 *    and the setup each signal wants, the run-time pin a Request carries
 *    as its select, the microsecond busy-wait `cs_setup_us` is spent on,
 *    and WHICH RATE IS SSPCLK - clk_peri (2.15.3.1), which
 *    rp2040/clock.hpp feeds from clk_sys undivided, so the bit rate
 *    comes from Clock::pclk_hz and never from a second statement of the
 *    rate;
 *  - the PUBLIC NAMES: `Pl022<n>` the resource, `SpiHost<n, pins, ...>`
 *    the engine `SpiBus` drives and `SpiClient<n, pins>` the other end,
 *    this chip's aliases of the three IP templates.
 *
 * WHAT THIS SILICON PUTS AROUND THE BLOCK. Two FIFOs of eight frames
 * each, so a host keeps up to eight frames in flight. The bit rate is
 * clk_peri / (CPSDVSR x (1 + SCR)): 62.5 Mbit/s at the top of a 125 MHz
 * clk_peri for a host, and a client - which needs clk_peri at least
 * twelve times its clock (4.4.3.4) - 10.4 Mbit/s here.
 *
 * SOD DOES NOT RELEASE THE PAD ON THIS CHIP (measured, the suite's wire
 * letters): the transmit line under SOD reads as a driven low, not as
 * the host's pull-up. The datasheet says why in passing, in 4.4.3.14's
 * list of idle levels - the block's pad-enable output nSSPOE IS NOT
 * CONNECTED TO THE PAD HERE - so SOD stops the block driving and leaves
 * the pad driving whatever it held. That is why
 * `SpiClient::drive_output(false)` releases the PAD, the other strata's
 * way, and SOD stays a bit of the resource; the IP file makes the pad
 * the lever for exactly this reason, so a silicon that does wire nSSPOE
 * to its pad needs no other code.
 *
 * THE PINS are fixed per instance under function 1 (2.19.2, table 279):
 * SPI0 receives on GPIO 0, 4, 16, 20, selects on 1, 5, 17, 21, clocks on
 * 2, 6, 18, 22 and transmits on 3, 7, 19, 23; SPI1 on 8, 12, 24, 28 /
 * 9, 13, 25, 29 / 10, 14, 26 / 11, 15, 27. `spi_pins_valid` is that
 * table. The host's chip select is NOT among them: it is a GPIO the
 * Request carries (the IP file says why), while the CLIENT's select IS
 * the pad.
 *
 * THE LINE. The four maskable sources of an instance combine into ONE
 * NVIC line (SPI0_IRQ, SPI1_IRQ, 2.3.2); the app binds isr_spi0 /
 * isr_spi1 to the instance's isr() body.
 *
 * THE DMA ENGINES ride any two channels, told the instance's requests
 * (Dreq::spi{n}_tx / spi{n}_rx, table 119) at arm(). The IP file raises
 * the requests around the engines and nowhere else, which is what this
 * chip's controller wants: a request standing with no channel behind it
 * banks credits the next block would spend on a FIFO that has not got
 * the data (rp2040/dma.hpp).
 */

#pragma once

#include <stdint.h>

#include "rp2040/device.hpp"

// This chip's busy-wait comes FIRST: the traits below hand the IP file
// `DelayRate`, a type of this family, and the driver spends `cs_setup_us`
// by calling that type's own verb - so the verb is in scope where the
// driver that calls it is defined.
#include "rp2040/delay.hpp"

#include "pl022/spi.hpp"
#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/resets.hpp"

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

// ---- what this chip tells the PL022 driver ----------------------------------------

/**
 * The chip traits brio/pl022/spi.hpp is instantiated with: every fact
 * about the PL022 that is the RP2040's rather than ARM's, and nothing
 * else. Nothing above this file ever names it.
 */
struct Rp2040Pl022 {
    Rp2040Pl022() = delete;

    using Regs = SPI0_Type;
    using Irq = IRQn_Type;
    using Interrupts = Nvic;
    using Pins = SpiPins;
    using PinRef = brio::PinRef;
    using DmaRequest = Dreq;
    using DelayRate = brio::DelayRate;

    /// SPI0 and SPI1, both with 8-deep FIFOs (4.4.2).
    static constexpr uint8_t instances = 2;
    static constexpr uint8_t fifo_depth = 8;
    /// What a pin set puts in a signal it does not route.
    static constexpr uint8_t no_pad = 0xFFu;

    template <uint8_t i>
    static Regs& regs() { return *(i == 0 ? SPI0 : SPI1); }
    template <uint8_t i>
    static constexpr Irq irq() { return i == 0 ? SPI0_IRQ_IRQn : SPI1_IRQ_IRQn; }

    /// The instance's bit in the subsystem reset controller (2.14).
    template <uint8_t i>
    static constexpr uint32_t reset_bit() { return i == 0 ? ResetBlock::spi0 : ResetBlock::spi1; }

    template <uint8_t i>
    static bool reset() { return Resets::cycle(reset_bit<i>()); }
    template <uint8_t i>
    static bool released() { return Resets::released(reset_bit<i>()); }
    template <uint8_t i>
    static void hold() { Resets::hold(reset_bit<i>()); }

    /// Table 119: the two data requests of an instance.
    template <uint8_t i>
    static constexpr DmaRequest tx_request() { return i == 0 ? Dreq::spi0_tx : Dreq::spi1_tx; }
    template <uint8_t i>
    static constexpr DmaRequest rx_request() { return i == 0 ? Dreq::spi0_rx : Dreq::spi1_rx; }

    /// The atomic register aliases of 2.1.2: one bus write, no
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
    /// (2.19.2: F1 on every SPI pin), and the electrical setup each
    /// signal wants. The two pulled ones are the HOST's receive - this
    /// chip's pads come up pulled DOWN, so an answer line with nothing
    /// driving it would read as zeros rather than as the idle ones a bus
    /// expects - and the CLIENT's select, so an unwired select reads
    /// not-selected. The client's receive is driven by the host whenever
    /// it matters and takes no pull.
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
    /// identity (rp2040/dma_engine.hpp).
    template <typename Tx, typename Rx>
    static constexpr bool engines_distinct() { return uart_engines_distinct<Tx, Rx>(); }

    /// The microsecond busy-wait of this core (rp2040/delay.hpp, over
    /// cortexm/delay.hpp): the factor for a rate, computed once per
    /// clock change so no division runs while a chip select settles.
    static constexpr DelayRate delay_rate(uint32_t hz) { return brio::delay_rate(hz); }

    /// SSPCLK is clk_peri (2.15.3.1) - `Clock::pclk_hz`, the one truth
    /// about the rate the bit rate is divided from.
    template <typename Clock>
    static constexpr uint32_t sspclk_hz(Clock) { return Clock::pclk_hz; }
};

static_assert(Pl022Chip<Rp2040Pl022>);

// ---- the public names --------------------------------------------------------------

/// The resource: which instance, where its registers are, its reset bit,
/// its NVIC line, and the whole of the PL022's programmer's model.
template <uint8_t n>
using Pl022 = Pl022Ssp<Rp2040Pl022, n>;

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
 * The two engine slots take rp2040/dma.hpp's DmaTxEngine / DmaRxEngine
 * on any two channels, both or neither.
 */
template <uint8_t n, SpiPins pins, typename TxEngine = NoDmaEngine, typename RxEngine = NoDmaEngine>
using SpiHost = Pl022Host<Rp2040Pl022, n, pins, TxEngine, RxEngine>;

/**
 * The other end of the wire, over Pl022<n>: the select goes to the
 * PERIPHERAL here (the PL022 in slave mode frames on SSPFSSIN).
 *
 *   constexpr brio::SpiPins client_pins{.sck = 10, .tx = 11, .rx = 8, .cs = 9};
 *   using Client = brio::SpiClient<1, client_pins>;
 *   Client::init(clock, {.mode = brio::SpiMode::mode1});
 */
template <uint8_t n, SpiPins pins>
using SpiClient = Pl022Client<Rp2040Pl022, n, pins>;

} // namespace brio
