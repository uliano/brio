#pragma once
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Bit-bang driver for the Microchip MCP3550/1/3 (22-bit single-channel delta-sigma
// ADC) on the AVR128DB28. There is no free hardware SPI on the 28-pin part (SPI1 is
// the ADS131M02 on PORTC; SPI0's only position PA4-7 is blocked by the LED on PA5
// and the ADS CLKIN on PA7), and the MCP3550 is slow (12.5-60 SPS) and read-only
// (no DIN), so 3 bit-banged GPIOs are plenty.
//
// Interface (MCP3550 datasheet Section 5.0, SPI mode 1,1): SCK idles HIGH; the
// device shifts a new bit out on each FALLING edge of SCK and the host latches it
// on the RISING edge, MSb first. SDO/RDY doubles as data and a busy/ready flag
// (low = ready) and is valid only while CS is low. CS held LOW = Continuous
// Conversion mode (the device free-runs at its fixed data rate).
//
// 24-bit frame, MSb first: [OVL b23][OVH b22][sign b21][data b20..b0].
//   - In range (-VREF <= VIN < VREF): OVL = OVH = 0 and the SIGN is bit 21, so the
//     value is a 22-bit two's complement in bits[21:0] - NOT a 24-bit one. Hence we
//     mask bits[21:0] and sign-extend from bit 21.
//   VIN = code / 2^21 * VREF.
//
//   using Adc = Mcp3550<Pin<'D',1>, Pin<'D',2>, Pin<'D',3>>;  // SCK, SDO/RDY, CS

template <typename SckPin, typename SdoPin, typename CsPin>
class Mcp3550 {
 public:
  struct Sample {
    int32_t code;  // signed 22-bit conversion code; VIN = code / 2^21 * VREF
    bool ovh;      // VIN > +VREF overflow
    bool ovl;      // VIN < -VREF overflow
    bool ok;       // false = timed out waiting for RDY (check wiring / device)
  };

  static void init() {
    SckPin::output();
    SckPin::set();         // SCK idles high (SPI mode 1,1)
    SdoPin::input();
    SdoPin::pullup(true);  // driven while CS low; pull-up covers the Hi-Z moments
    CsPin::output();
    CsPin::set();           // CS idle high; read() pulses it low per conversion
  }

  // Single-shot read: a HIGH->LOW edge on CS starts a fresh conversion, then we
  // wait for SDO/RDY to fall (ready, ~tCONV: 80 ms on the -50) and clock the
  // 24-bit frame. (This part does NOT free-run in continuous mode here, so each
  // read triggers its own conversion - keeps the data fresh.) The default timeout
  // (~0.6 s at 24 MHz) covers the slowest variant yet bails if nothing answers.
  static Sample read(uint32_t timeout = 1500000u) {
    Sample s{0, false, false, false};
    CsPin::set();
    _delay_us(20);                      // CS high: end the previous frame / shutdown
    CsPin::clear();                     // CS HIGH->LOW edge starts a new conversion
    while (SdoPin::read()) {            // high = busy converting, low = ready
      if (--timeout == 0u) return s;    // RDY never came -> ok stays false
    }
    uint32_t raw = 0;
    for (uint8_t i = 0; i < 24; ++i) {
      SckPin::clear();                  // falling edge -> device shifts out a bit
      _delay_us(1);
      SckPin::set();                    // rising edge -> latch
      raw = (raw << 1) | (SdoPin::read() ? 1u : 0u);
      _delay_us(1);
    }

    s.ovl = (raw & (1UL << 23)) != 0;
    s.ovh = (raw & (1UL << 22)) != 0;
    int32_t code = static_cast<int32_t>(raw & 0x3FFFFFUL);  // 22-bit field [21:0]
    if (code & 0x200000L) code |= ~0x3FFFFFL;               // sign-extend bit 21
    s.code = code;
    s.ok = true;
    return s;
  }
};
