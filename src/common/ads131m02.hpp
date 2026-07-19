#pragma once
#include <stddef.h>
#include <stdint.h>

// ADS131M02 (TI, 2-ch 24-bit delta-sigma ADC) minimal register-level driver.
//
// Protocol logic adapted from uliano/blackpill-experiments
// (src/common/ads131m02.hpp); the wiring assumptions match this AVR128DB28
// board (see uliano/avr128db28-blink, which read ID=0x2205 / RESET=0xFF22):
//   - SPI1 on PC0(DIN) / PC1(DOUT) / PC2(SCLK), Mode 1, 3 MHz.
//   - CLKIN on PA7 (TCD0, 4 MHz) - must run before the ADC responds.
//   - CS is HARDWIRED to GND (always selected): no CS toggling. The frame
//     boundary is the idle SCLK gap after each 12-byte frame.
//   - SYNC/RESET is tied high: reset is via the SPI RESET command, not the pin.
//
// Protocol facts (datasheet SBAS853A): 24-bit words = 16-bit payload MSB-aligned
// + 1 pad byte; an M02 frame is 4 words (resp + ch0 + ch1 + CRC) = 12 bytes; a
// command's response appears in the FOLLOWING frame; input CRC is OFF by default
// so commands carry no CRC; output CRC is always present but we ignore it here.

namespace ads131 {

// Register addresses (subset; full map in the datasheet / firmware digest).
enum class Reg : uint8_t {
  ID = 0x00,
  STATUS = 0x01,
  MODE = 0x02,
  CLOCK = 0x03,
  GAIN = 0x04,
  CFG = 0x06,
  CH0_CFG = 0x09,       // per-channel config; MUX[1:0]: 00=AINxP/N, 01=inputs shorted
  CH0_OCAL_MSB = 0x0A,  // 0x0A/0x0B OCAL, 0x0C/0x0D GCAL (MSB then LSB)
  CH0_GCAL_MSB = 0x0C,
  CH1_CFG = 0x0E,
  CH1_OCAL_MSB = 0x0F,  // 0x0F/0x10 OCAL, 0x11/0x12 GCAL
  CH1_GCAL_MSB = 0x11,
};

// Per-channel hardware calibration, in raw OCAL/GCAL register units. The device applies
// it to EVERY conversion (always active): OUT = (RAW - OCAL) * GCAL / 2^23.
//   ocal: 24-bit two's complement, subtracted (offset correction).
//   gcal: 24-bit unsigned, multiplies; kGcalUnity (0x800000) = factor 1.0.
struct Calibration {
  int32_t ocal0;
  uint32_t gcal0;
  int32_t ocal1;
  uint32_t gcal1;
};

constexpr uint32_t kGcalUnity = 0x800000;   // GCAL for gain factor 1.0 (= 2^23)
constexpr int32_t kNomFullScale = 8388608;  // 2^23: ADC nominal full-scale (VREF/gain)

// GCAL word that makes a channel read true volts on the NOMINAL VREF scale, from the
// measured span (= code@VREF - code@0V) for an applied reference v_ref_uV. Keeps the
// usual V = code * v_ref_nom_uV / 2^23 valid in the apps (default VREF nominal 1.2 V).
constexpr uint32_t gcal_for_span(int32_t span, uint32_t v_ref_uV = 1024000,
                                 uint32_t v_ref_nom_uV = 1200000) {
  // GCAL = 2^23 * nominal_span / span, nominal_span = v_ref * 2^23 / v_ref_nom.
  // Single return statement to satisfy C++11 constexpr rules.
  return static_cast<uint32_t>(
      static_cast<int64_t>(kGcalUnity) *
      (static_cast<int64_t>(v_ref_uV) * kNomFullScale / v_ref_nom_uV) / span);
}

// Build a Calibration directly from an adccal two-point run: internal-short offsets
// c0_* and the codes cR_* captured at the applied reference.
constexpr Calibration cal_from_two_point(int32_t c0_0, int32_t cR_0, int32_t c0_1,
                                         int32_t cR_1, uint32_t v_ref_uV = 1024000) {
  return Calibration{c0_0, gcal_for_span(cR_0 - c0_0, v_ref_uV),
                     c0_1, gcal_for_span(cR_1 - c0_1, v_ref_uV)};
}

// 16-bit command opcodes (placed in the MSBs of a 24-bit word).
constexpr uint16_t kCmdNull = 0x0000;    // response = STATUS
constexpr uint16_t kCmdReset = 0x0011;   // response (next frame) = 0xFF22
constexpr uint16_t kCmdUnlock = 0x0655;  // unlock the interface for WREG
constexpr uint16_t kResetAck = 0xFF22;
constexpr uint8_t kIdHigh = 0x22;        // ID high byte identifies the device

template <typename SpiT>
class Ads131m02 {
 public:
  // SPI must already be initialized by the app (SpiT::init()).
  static void init() { SpiT::init(); }

  // RESET command (full frame) -> ack read in the FOLLOWING frame (0xFF22).
  // Caller must then wait tREGACQ before reading registers.
  static uint16_t reset() {
    send(kCmdReset);
    return send(kCmdNull);
  }

  // UNLOCK the register interface (required before WREG). Returns the ack 0x0655.
  static uint16_t unlock() {
    send(kCmdUnlock);
    return send(kCmdNull);
  }

  // RREG (single) -> register value comes back in the FOLLOWING frame.
  static uint16_t read_reg(Reg r) {
    send(rreg_opcode(r));
    return send(kCmdNull);
  }

  // WREG (single) -> data word travels right after the opcode in the same frame.
  static void write_reg(Reg r, uint16_t value) {
    const uint16_t tx[kFrameWords] = {wreg_opcode(r), value, 0, 0};
    uint16_t rx[kFrameWords];
    frame(tx, rx);
  }

  // Load per-channel offset/gain into the always-active OCAL/GCAL registers. Call once
  // after reset() (which restores their defaults); it unlocks the interface first. The
  // device then corrects every conversion: OUT = (RAW - OCAL) * GCAL / 2^23, so apps
  // keep their nominal V = code * VREF / 2^23 and get true volts.
  static void load_calibration(const Calibration& c) {
    unlock();
    write_cal24(Reg::CH0_OCAL_MSB, static_cast<uint32_t>(c.ocal0));
    write_cal24(Reg::CH0_GCAL_MSB, c.gcal0);
    write_cal24(Reg::CH1_OCAL_MSB, static_cast<uint32_t>(c.ocal1));
    write_cal24(Reg::CH1_GCAL_MSB, c.gcal1);
  }

  // One conversion frame (send NULL): STATUS + the two FULL 24-bit signed channel
  // results (the register path keeps only 16 bits; conversion data keeps 24).
  struct Data {
    int32_t ch0;
    int32_t ch1;
    uint16_t status;
  };
  static Data read_data() {
    const uint16_t tx[kFrameWords] = {kCmdNull, 0, 0, 0};
    uint8_t tb[kFrameWords * 3];
    uint8_t rb[kFrameWords * 3];
    pack(tx, tb);
    SpiT::transfer(tb, rb, kFrameWords * 3);
    settle();
    Data d;
    d.status = static_cast<uint16_t>((rb[0] << 8) | rb[1]);
    d.ch0 = signed24(rb[3], rb[4], rb[5]);
    d.ch1 = signed24(rb[6], rb[7], rb[8]);
    return d;
  }

 private:
  static constexpr size_t kFrameWords = 4;  // resp/cmd + ch0 + ch1 + CRC (M02)

  // Write a 24-bit cal value across its MSB (bits 23..8) + LSB (bits 7..0 in [15:8]).
  static void write_cal24(Reg msb, uint32_t v) {
    write_reg(msb, static_cast<uint16_t>((v >> 8) & 0xFFFFu));
    const Reg lsb = static_cast<Reg>(static_cast<uint8_t>(msb) + 1);
    write_reg(lsb, static_cast<uint16_t>((v & 0xFFu) << 8));
  }

  // Sign-extend a big-endian 24-bit two's-complement word to int32.
  static int32_t signed24(uint8_t b0, uint8_t b1, uint8_t b2) {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(b0) << 16) |
                                     (static_cast<uint32_t>(b1) << 8) | b2);
    if (b0 & 0x80u) v |= static_cast<int32_t>(0xFF000000u);
    return v;
  }

  static constexpr uint16_t rreg_opcode(Reg r) {
    return 0xA000u | (static_cast<uint16_t>(static_cast<uint8_t>(r) & 0x3Fu) << 7);
  }
  static constexpr uint16_t wreg_opcode(Reg r) {
    return 0x6000u | (static_cast<uint16_t>(static_cast<uint8_t>(r) & 0x3Fu) << 7);
  }

  // Clock one command word (rest of the frame = 0) and return word 0 of the
  // response (= the answer to the command from the PREVIOUS frame).
  static uint16_t send(uint16_t command) {
    const uint16_t tx[kFrameWords] = {command, 0, 0, 0};
    uint16_t rx[kFrameWords];
    frame(tx, rx);
    return rx[0];
  }

  // Pack 4 device words into 12 bytes: each 24-bit word = 16-bit payload + pad.
  static void pack(const uint16_t* w, uint8_t* b) {
    for (size_t i = 0; i < kFrameWords; ++i) {
      b[3 * i] = static_cast<uint8_t>(w[i] >> 8);
      b[3 * i + 1] = static_cast<uint8_t>(w[i] & 0xFF);
      b[3 * i + 2] = 0;  // 3rd (pad) byte of the 24-bit word
    }
  }

  // One 12-byte frame. CS is hardwired low, so we just clock the bytes and then
  // hold SCLK idle briefly so the device recognizes the frame boundary.
  static void frame(const uint16_t* tx, uint16_t* rx) {
    uint8_t tb[kFrameWords * 3];
    uint8_t rb[kFrameWords * 3];
    pack(tx, tb);
    SpiT::transfer(tb, rb, kFrameWords * 3);
    settle();
    for (size_t i = 0; i < kFrameWords; ++i) {
      rx[i] = static_cast<uint16_t>((rb[3 * i] << 8) | rb[3 * i + 1]);
    }
  }

  // ~5 us idle gap between frames (SCLK low) - the inter-frame delimiter.
  static void settle() {
    for (volatile uint8_t i = 0; i < 40; ++i) {
    }
  }
};

}  // namespace ads131
