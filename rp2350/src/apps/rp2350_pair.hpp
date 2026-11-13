// rp2350_pair.hpp - the desk pair's links as the board chooses them, the
// RP2350 end: ONE statement each for the SPI link, the I2C bus, the serial
// link and the clock pair, read by the peers (spi_peer, twi_peer,
// uart_peer), by the suites that drive them, by the bench apps and by
// pair_power.hpp, so that no app writes a link pad's number. App-level
// bench tooling; nothing in brio knows it exists. Constants only: no
// function here forms code.
//
// THE PLAN IS THE BOARD'S, keyed on the board the build is for
// (board::type, from the board file board.hpp). The far end is the desk's
// WeAct RP2040 in both plans, on the same pads (docs/boards/weact-rp2040.md):
// SPI1 GP10 SCK, GP11 its TX, GP8 its RX, GP29 the select; I2C1 GP2 SDA,
// GP3 SCL, each line 1.5 kOhm to its 3V3; UART1 GP4 TX, GP5 RX; GPOUT0 on
// GP21 and GPIN0 on GP20. Only this end moves:
//
//   link            WeAct RP2350A (QFN-60)   WeAct RP2350B (QFN-80)
//   SPI1 SCK        GP10                     GP30
//   SPI1 RX         GP8                      GP28
//   SPI1 TX         GP11 (47 Ohm here)       GP31 (47 Ohm here)
//   SPI1 CSn        GP9                      GP41
//   I2C0 SDA        GP16                     GP36
//   I2C0 SCL        GP17                     GP37
//   UART1 TX        GP4                      GP4
//   UART1 RX        GP5                      GP5
//   GPOUT0          GP21                     GP21
//   GPIN0           GP20                     GP20
//
// Every pad above is checked against the RP2350 datasheet's function table
// (9.4, table 677) by a static_assert below, through the stratum's own pad
// predicates: GP8 SPI1 RX, GP9 SPI1 CSn, GP10 SPI1 SCK, GP11 SPI1 TX,
// GP16/GP17 I2C0 SDA/SCL, GP4/GP5 UART1 TX/RX under function 2, GP20
// CLOCK GPIN0 and GP21 CLOCK GPOUT0 under function 9 - and GP28, GP30,
// GP31, GP41, GP36 and GP37 the same signals on the QFN-80.
//
// - THE WEACT RP2350B, `weact2350b`: SPI1 and I2C0 in the high pads; GP29
//   does not drive on that board (its page), which is why the select is on
//   GP41, and the I2C link sits above GP31 so that its pads are the high
//   half of the bank.
// - THE WEACT RP2350A, `weact2350a`: the QFN-60 stops at GP29, and its
//   board takes GP23, GP24 and GP29 for its own circuits, so both links
//   sit in the low pads: SPI1's group GP8..GP11 whole, I2C0 on GP16/GP17.
//
// THE PADS THE PLAN LEAVES TO THE BENCH'S OWN INSTRUMENTS, at the end: the
// SPI0 host the loop-back letters run (no wire on its pads) moves off the
// A's I2C link.

#pragma once

#include <stdint.h>

#include "board.hpp"
#include "rp2350/clock.hpp"
#include "rp2350/i2c.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/spi.hpp"
#include "rp2350/uart.hpp"

namespace rp2350pair {

static_assert(board::is("weact2350a") || board::is("weact2350b"),
              "rp2350_pair.hpp: no pair plan for this board - the plans are the WeAct RP2350A's and RP2350B's");

/// Whether the build is for the WeAct RP2350A: the one question the plan
/// asks of the board.
inline constexpr bool board_a = board::is("weact2350a");

// ---- THE SPI LINK: SPI1, the RP2040's SPI1 at the far end ----------------

inline constexpr uint8_t spi_sck = board_a ? 10u : 30u;
/// This chip's receive pad: the far end's transmit line.
inline constexpr uint8_t spi_rx = board_a ? 8u : 28u;
/// This chip's transmit pad, 47 Ohm in series at this end.
inline constexpr uint8_t spi_tx = board_a ? 11u : 31u;
/// The select: 4.7 kOhm to the RP2040's 3V3.
inline constexpr uint8_t spi_cs = board_a ? 9u : 41u;
static_assert(brio::spi_sck_pin(1, spi_sck) && brio::spi_rx_pin(1, spi_rx) && brio::spi_tx_pin(1, spi_tx) &&
                  brio::spi_cs_pin(1, spi_cs),
              "RP2350 datasheet table 677: the SPI link's four pads are SPI1's SCK, RX, TX and CSn (function 1)");
static_assert(spi_sck < brio::gpio_count && spi_rx < brio::gpio_count && spi_tx < brio::gpio_count &&
                  spi_cs < brio::gpio_count,
              "the SPI link's pads are bonded in the package the board carries");

/// The host on the link: the select is a plain pad the suite drives.
inline constexpr brio::SpiPins spi_host_pins{.sck = spi_sck, .tx = spi_tx, .rx = spi_rx};
/// The PL022 client (or host_burst's host): the select is the block's own.
inline constexpr brio::SpiPins spi_client_pins{.sck = spi_sck, .tx = spi_tx, .rx = spi_rx, .cs = spi_cs};

/// A PIO block's window over the link's pads (GPIOBASE, datasheet 11.1.1):
/// a block sees 32 pads from its base, 0 or 16, so a pad above GP31 needs
/// the window at 16 and the A's pads, all below GP16, the window at 0.
inline constexpr uint8_t spi_pio_window_base =
    (spi_sck > 31u || spi_rx > 31u || spi_tx > 31u || spi_cs > 31u) ? 16u : 0u;
static_assert(spi_sck >= spi_pio_window_base && spi_rx >= spi_pio_window_base &&
                  spi_tx >= spi_pio_window_base && spi_cs >= spi_pio_window_base &&
                  spi_sck < spi_pio_window_base + 32u && spi_rx < spi_pio_window_base + 32u &&
                  spi_tx < spi_pio_window_base + 32u && spi_cs < spi_pio_window_base + 32u,
              "the SPI link's four pads fit one PIO window");

// ---- THE I2C BUS: I2C0, the RP2040's I2C1 at the far end ------------------

/// Each line 1.5 kOhm to the RP2040's 3V3 and to nothing else: the pull-ups
/// pair_power.hpp reads the partner's rail through.
inline constexpr uint8_t i2c_sda = board_a ? 16u : 36u;
inline constexpr uint8_t i2c_scl = board_a ? 17u : 37u;
inline constexpr brio::I2cPins i2c_pins{.scl = i2c_scl, .sda = i2c_sda};
static_assert(brio::i2c_pins_valid(0, i2c_pins) && i2c_sda < brio::gpio_count && i2c_scl < brio::gpio_count,
              "RP2350 datasheet table 677: the I2C link's pads are I2C0's SDA and SCL (function 3)");
/// Whether the bus sits in the high half of the bank, above GP31: the B's
/// plan, on purpose; the A's package has no high half.
inline constexpr bool i2c_in_high_half = i2c_sda > 31u && i2c_scl > 31u;

// ---- THE SERIAL LINK: UART1, the RP2040's UART1 at the far end ------------

inline constexpr uint8_t uart_tx = 4u;
inline constexpr uint8_t uart_rx = 5u;
inline constexpr brio::UartPins uart_pins{
    .tx = {uart_tx, brio::PinFunction::uart},
    .rx = {uart_rx, brio::PinFunction::uart},
};
static_assert(brio::uart_pins_valid(1, uart_pins),
              "RP2350 datasheet table 677: the serial link's pads are UART1's TX and RX under function 2");

// ---- THE CLOCK PAIR: this GPOUT0 into the RP2040's GPIN0, and back -------

/// The clock output and input on the link, by instance.
inline constexpr uint8_t clock_out = 0u;
inline constexpr uint8_t clock_in = 0u;
/// Their pads. The serial suites and their peers use the same two wires as
/// plain lines: the driving board's GPOUT0 pad into the peer's GPIN0 pad
/// says which role the peer takes.
inline constexpr uint8_t gpout = brio::gpout_pin(clock_out);
inline constexpr uint8_t gpin = brio::gpin_pin(clock_in);
static_assert(gpout == 21u && gpin == 20u,
              "RP2350 datasheet table 677: CLOCK GPOUT0 is GP21 and CLOCK GPIN0 GP20 (function 9)");

/// Every link pad, as two masks of the bank's halves (bit 0 of the high one
/// is GP32): what a program that waves pads leaves alone while the partner
/// is unpowered.
constexpr uint32_t pad_bit(uint8_t pad, bool high) {
    return (pad > 31u) == high ? (1UL << (pad & 31u)) : 0u;
}
constexpr uint32_t link_mask(bool high) {
    return pad_bit(spi_sck, high) | pad_bit(spi_rx, high) | pad_bit(spi_tx, high) | pad_bit(spi_cs, high) |
           pad_bit(i2c_sda, high) | pad_bit(i2c_scl, high) | pad_bit(uart_tx, high) | pad_bit(uart_rx, high) |
           pad_bit(gpout, high) | pad_bit(gpin, high);
}
inline constexpr uint32_t link_low = link_mask(false);
inline constexpr uint32_t link_high = link_mask(true);

// ---- THE PADS LEFT TO THE BENCH'S OWN INSTRUMENTS --------------------------

/// THE SPI0 LOOP-BACK HOST of the SPI suite and the bench app, with no
/// wire on its pads (the PL022's LBM feeds its transmitter into its
/// receiver inside the block): SCK GP18 and TX GP19 on both boards. On the
/// B its receive pad is GP16 and its select GP17; on the A those two are
/// the I2C link's and every other SPI0 receive pad the QFN-60 has carries
/// the console (GP0), the serial link (GP4) or the clock input (GP20), so
/// the host has no receive pad there - the loop-back never needs one - and
/// its select is GP26, a free pad of the A's header.
inline constexpr brio::SpiPins loop_host_pins =
    board_a ? brio::SpiPins{.sck = 18u, .tx = 19u} : brio::SpiPins{.sck = 18u, .tx = 19u, .rx = 16u};
inline constexpr uint8_t loop_select = board_a ? 26u : 17u;
static_assert(brio::spi_pins_valid(0, loop_host_pins) && (link_low & (1UL << loop_select)) == 0u &&
                  (link_low & ((1UL << loop_host_pins.sck) | (1UL << loop_host_pins.tx))) == 0u &&
                  (loop_host_pins.rx == 0xFFu || (link_low & (1UL << loop_host_pins.rx)) == 0u),
              "the SPI0 loop-back host's pads are SPI0's and none of them is a link pad");

}  // namespace rp2350pair
