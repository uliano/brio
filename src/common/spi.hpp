#pragma once
#include <avr/io.h>
#include <stddef.h>
#include <stdint.h>

// Minimal SPI master drivers for the AVR128DB28, register-level (no framework).
//
// On the 28-pin part each SPI has a single fixed pin position:
//   SPI0 default -> PA4(MOSI) PA5(MISO) PA6(SCK) PA7(SS)
//   SPI1 default -> PC0(MOSI) PC1(MISO) PC2(SCK) PC3(SS)
//
// The ADS131M02 board uses SPI1 on PORTC (PC0-PC2), with PC3 freed as a plain
// GPIO (DRDY) by disabling hardware SS (SSD). Mode 1 (CPOL=0, CPHA=1), 3 MHz at
// CLK_PER=24 MHz. These values reproduce the proven avr128db28-blink setup
// (SPI1.CTRLA=0x33, SPI1.CTRLB=0x05).

namespace spi {

// SPI0 on PORTA, master, fCLK_PER/8 = 3 MHz, SPI mode selectable. All-static.
// The SPI mode picks SCK polarity/phase for the attached device:
//   mode 0 (CPOL=0,CPHA=0): SCK idles LOW, sample on rising  -> MCP4921 DAC
//   mode 3 (CPOL=1,CPHA=1): SCK idles HIGH, sample on rising -> MCP3550 ADC
// PA7 (the default SS) is left free by disabling hardware SS (SSD). MOSI (PA4) and
// SCK (PA6) are outputs; MISO (PA5) is an input and stays readable as a plain GPIO
// (via VPORTA.IN) while SPI is enabled - used by the MCP3550 to poll its SDO/RDY
// busy flag. The MCP4921 is write-only and ignores MISO.
class Spi0 {
 public:
  static void init(uint8_t mode = 0) {
    // SPI0 default pin position (PA4-PA7): leave the PORTMUX SPI0 field at 0.
    PORTA.DIRSET = PIN4_bm | PIN6_bm;   // MOSI (PA4), SCK (PA6) as outputs
    PORTA.DIRCLR = PIN5_bm;             // MISO (PA5) as input
    PORTA.PIN5CTRL = PORT_PULLUPEN_bm;  // pull-up on MISO (covers SDO Hi-Z moments)

    // MODE[1:0] from the argument + SS disabled (PA7 stays a free GPIO).
    SPI0.CTRLB = SPI_SSD_bm | (mode & 0x03);
    // Enable, master, prescaler /16 with CLK2X -> /8 = 3 MHz. == 0x33.
    SPI0.CTRLA = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_PRESC_DIV16_gc | SPI_CLK2X_bm;
  }

  // Full-duplex single byte (blocking on the receive-complete flag).
  static uint8_t transfer(uint8_t b) {
    SPI0.DATA = b;
    while (!(SPI0.INTFLAGS & SPI_RXCIF_bm)) {}
    return SPI0.DATA;
  }

  // Full-duplex block transfer. tx==nullptr sends zeros; rx==nullptr discards.
  static void transfer(const uint8_t* tx, uint8_t* rx, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      const uint8_t in = transfer(tx ? tx[i] : 0x00);
      if (rx) rx[i] = in;
    }
  }
};

// SPI1 on PORTC, master, Mode 1, fCLK_PER/8 = 3 MHz. All-static, no state.
class Spi1 {
 public:
  static void init() {
    // SPI1 default pin position (PC0-PC3): leave the PORTMUX SPI1 field at 0.
    PORTC.DIRSET = PIN0_bm | PIN2_bm;   // MOSI (PC0), SCK (PC2) as outputs
    PORTC.DIRCLR = PIN1_bm;             // MISO (PC1) as input
    PORTC.PIN1CTRL = PORT_PULLUPEN_bm;  // pull-up on MISO

    // Mode 1 (MODE[1:0]=01) + SS disabled (PC3 stays a free GPIO for DRDY).
    SPI1.CTRLB = SPI_SSD_bm | 0x01;
    // Enable, master, prescaler /16 with CLK2X -> /8 = 3 MHz. == 0x33.
    SPI1.CTRLA = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_PRESC_DIV16_gc | SPI_CLK2X_bm;
  }

  // Full-duplex single byte (blocking on the receive-complete flag).
  static uint8_t transfer(uint8_t b) {
    SPI1.DATA = b;
    while (!(SPI1.INTFLAGS & SPI_RXCIF_bm)) {}
    return SPI1.DATA;
  }

  // Full-duplex block transfer. tx==nullptr sends zeros; rx==nullptr discards.
  static void transfer(const uint8_t* tx, uint8_t* rx, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      const uint8_t in = transfer(tx ? tx[i] : 0x00);
      if (rx) rx[i] = in;
    }
  }
};

}  // namespace spi
