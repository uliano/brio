#pragma once
#include <avr/io.h>
#include <stdint.h>

// Driver for the Microchip MCP4921 (single 12-bit SPI DAC) on the AVR128DB28.
// Write-only: the device has no data output (SDO), so it shares SCK (PA6) and
// MOSI/SDI (PA4) on SPI0 with any other DAC and only needs its own CS line.
//
// The MCP4921 wants SPI mode 0 (SCK idles low, data clocked in on the rising edge),
// so initialise the shared bus with spi::Spi0::init(0). LDAC is assumed tied to GND
// on this board, so the output updates on the CS rising edge at the end of write().
//
// 16-bit command word, MSb first (datasheet Section 5.0):
//   bit15 A/B  = 0  (DACA - the only channel on the 4921)
//   bit14 BUF  = 0  (VREF input unbuffered: full 0..VREF range, ref drives the ladder)
//   bit13 GA   = 1  (gain 1x: VOUT = VREF * code / 4096; GA=0 would be 2x)
//   bit12 SHDN = 1  (DAC active; 0 = output buffer disabled / high-Z)
//   bit11..0   = 12-bit code
//
//   using DacA = Mcp4921<spi::Spi0, Pin<'D',3>>;  // CS on PD3
template <typename Spi, typename CsPin>
class Mcp4921 {
 public:
  static constexpr uint16_t kConfig = 0x3000;  // BUF=0, GA=1 (1x), SHDN=1

  static void init() {
    CsPin::output();
    CsPin::set();  // CS idle high
  }

  // Load a 12-bit code (0..4095). VOUT = VREF * code / 4096 (gain 1x).
  static void write(uint16_t code) {
    const uint16_t cmd = kConfig | (code & 0x0FFF);
    CsPin::clear();
    Spi::transfer(static_cast<uint8_t>(cmd >> 8));
    Spi::transfer(static_cast<uint8_t>(cmd & 0xFF));
    CsPin::set();  // CS rising latches the word; LDAC=GND -> VOUT updates now
  }
};
