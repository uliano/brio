// ch32_pair.hpp - the desk pair's three links as the board chooses them:
// ONE statement each for the I2C bus, the SPI link and the serial link,
// read by the peers (twi_peer, spi_peer, usart_peer) and by the suites that
// drive them (test_ch32_i2c, test_ch32_spi, test_ch32_serial, the bench
// apps), so the two ends of a link are chosen in one place. App-level bench
// tooling; nothing in brio knows it exists. Constants only: no function
// here forms code.
//
// THE PLAN IS THE BOARD'S, and the board key is the package (ch32_board.hpp
// says why and which board sits on each key). The far end is the desk's
// CH32X035 in every plan, on its SPI1 PA4..PA7, its I2C1 PC16/PC17 (the
// pull-ups its own, 1.5 kOhm from its PB3) and its USART4 PB0/PB1.
//
// - THE WEACT BOARD (the CH32V006's 20-pin package, no port B):
//     SPI1 column 1   NSS PC0, SCK PC5, MISO PC7, MOSI PC6  (AFIO table 7-12)
//     I2C1 column 0   SCL PC2, SDA PC1                      (table 7-13-1)
//     USART2 column 3 TX PD2, RX PD3                        (table 7-11)
// - THE CH32V006K8U6 MODULE (the QFN32):
//     SPI1 column 0   NSS PC1, SCK PC5, MISO PC7, MOSI PC6
//     I2C1 column 3   SCL PB5, SDA PB6 - SPI1's column 0 left whole
//     USART2 column 4 TX PB0, RX PB1
// - THE CH32V003 (off the desk): SPI1 column 0, I2C1's default pads, and
//   no serial link - its one USART is the console.

#pragma once

#include <stdint.h>

#include "ch32_board.hpp"
#include "ch32v00x/device.hpp"
#include "ch32v00x/i2c.hpp"
#include "ch32v00x/spi.hpp"
#include "ch32v00x/usart.hpp"

namespace ch32pair {

/// THE SPI LINK: SPI1's column, every signal of it.
inline constexpr uint8_t spi_column = ch32board::weact ? 1 : 0;
inline constexpr brio::SpiPins spi_pins = brio::spi1_pins_for(spi_column);
static_assert(spi_pins.sck == brio::Pad{'C', 5} && spi_pins.mosi == brio::Pad{'C', 6} &&
                  spi_pins.miso == brio::Pad{'C', 7} &&
                  spi_pins.nss == (ch32board::weact ? brio::Pad{'C', 0} : brio::Pad{'C', 1}),
              "AFIO table 7-12: SPI1's codes 000 and 001 differ in NSS alone, PC1 and PC0");

/// THE I2C BUS: I2C1's column.
inline constexpr brio::I2cPins i2c_pins =
    brio::device::part == brio::Ch32Part::v006 && !ch32board::weact ? brio::i2c1_pins_for(3)
                                                                    : brio::i2c1_default_pins;
static_assert(brio::device::part != brio::Ch32Part::v006 || ch32board::weact ||
                  (i2c_pins.scl == brio::Pad{'B', 5} && i2c_pins.sda == brio::Pad{'B', 6}),
              "AFIO table 7-13-1: I2C1's code 011 is SCL PB5, SDA PB6");
static_assert(!ch32board::weact || (i2c_pins.scl == brio::Pad{'C', 2} && i2c_pins.sda == brio::Pad{'C', 1}),
              "AFIO table 7-13-1: I2C1's code 000 is SCL PC2, SDA PC1");

/// THE SERIAL LINK: USART2's column, zero where the part has no USART2.
inline constexpr uint8_t serial_column = !brio::device::has_usart2 ? 0 : ch32board::weact ? 3 : 4;
static_assert(serial_column == 0u ||
                  (brio::usart_pads_bonded(2, serial_column) &&
                   brio::afio_usart2_pads(serial_column).tx ==
                       (ch32board::weact ? brio::Pad{'D', 2} : brio::Pad{'B', 0}) &&
                   brio::afio_usart2_pads(serial_column).rx ==
                       (ch32board::weact ? brio::Pad{'D', 3} : brio::Pad{'B', 1})),
              "AFIO table 7-11: USART2's code 011 is TX PD2, RX PD3, code 100 TX PB0, RX PB1");

}  // namespace ch32pair
