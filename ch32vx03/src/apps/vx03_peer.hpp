// vx03_peer.hpp - what every bench peer of this stratum shares: the time
// base, the ident label and the die on the banner. App-level bench
// tooling beside the peers that include it, as the link protocols sit
// beside theirs; nothing in brio knows it exists.
//
// THE TIME BASE IS THE CRYSTAL. Both desk boards of this stratum carry
// an 8 MHz crystal on OSC_IN/OSC_OUT, and a peer is an instrument: its
// rates are the far board's reference, and the HSI of the two bench dies
// runs a quarter to a half per cent fast (docs/ch32vx03/clock.md's bench
// findings). So a peer's PLL is
// fed by the HSE, x12 to 96 MHz of HCLK, and Ident::xtal says the crystal
// started; without it the HSI's PLL runs the same 96 MHz nominal and the
// ident says 0.
//
// WHY 96 MHz. I2C's FREQ cannot state a PB1 above 60 MHz (RM 19.12.2),
// so a peer with an I2C keeps HCLK at or below 120 MHz, and 96 MHz puts
// PB1 at 48 (CCR exact at both speeds) and the timers of both buses at 96
// MHz - FOUR TIMES the 24 MHz tick usart_link.hpp counts time in, so a
// timer at prescaler 3 counts protocol ticks exactly (the static_asserts
// below). The SPI peer and the serial peer, which have no I2C, run the
// same crystal at 144 MHz instead (their time bases below: a client's bus
// at twice the fastest host's SCK, a port's at the far port's 4.5 Mbaud).
//
// THE LABEL is the part number after its CH32 prefix ("V203C8",
// "V303VC"), NUL padded to eight characters: with two parts of this
// stratum on one bus, WHICH PART answers is what a DUT reads, and the
// CLASS follows from it (CH32V20x_D6, CH32V30x_D8). The die - the chip
// identifier and the UID - is on the peer's console banner instead.

#pragma once

#include <stdint.h>

#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/nvm.hpp"
#include "util/print.hpp"

namespace vx03peer {

/// The crystal's 8 MHz times twelve; the HSI's PLL the fallback at the
/// same rate, so every divisor computed for the one holds for the other.
using PeerClock = brio::Clock<brio::ClockSource::pll, 96'000'000, 8'000'000>;
using HsiClock = brio::Clock<brio::ClockSource::pll, 96'000'000>;
static_assert(PeerClock::hz == HsiClock::hz && PeerClock::pclk1_hz == HsiClock::pclk1_hz &&
                  PeerClock::pclk2_hz == HsiClock::pclk2_hz,
              "the fallback must run every divisor at the same rate");
static_assert(PeerClock::pclk1_hz <= 60'000'000u,
              "RM 19.12.2: FREQ states PB1 in whole megahertz, 4 to 60");

/// usart_link.hpp's tick, and the prescaler that makes a timer of either
/// bus count it.
inline constexpr uint32_t tick_hz = 24'000'000;
static_assert(PeerClock::timclk1_hz % tick_hz == 0u && PeerClock::timclk2_hz % tick_hz == 0u,
              "a timer of either bus counts the protocol's 24 MHz tick at a whole prescaler");
inline constexpr uint16_t tick_prescaler = static_cast<uint16_t>(PeerClock::timclk1_hz / tick_hz - 1u);
static_assert(PeerClock::timclk1_hz == PeerClock::timclk2_hz,
              "one prescaler for the timers of both buses");

/// Start the tree: the crystal's PLL, else the HSI's. True in `crystal`
/// when the HSE started; the return says whether a clock runs at all.
inline bool start_clock(bool& crystal) {
    crystal = PeerClock::init();
    return crystal || HsiClock::init();
}

/// THE SPI PEER'S TIME BASE: the same crystal x18 to 144 MHz, the
/// stratum's ceiling, with the HSI's PLL the fallback at the same rate. A
/// client's shifter runs on its host's SCK, but its block needs a bus at
/// least twice that SCK (the CH32V203 datasheet's table 4-25 and the
/// CH32V303's table 4-27 give a client 36 MHz on a PB1 instance), and the
/// fastest host on this desk is a PB1 instance at /2 of 72 MHz - so the
/// peer's PB1 must run at 72 MHz too, which is HCLK 144 (PB1 capped at 72,
/// docs/ch32vx03/clock.md). The SPI peer has no I2C, so 19.12.2's ceiling
/// does not bind it.
using SpiPeerClock = brio::Clock<brio::ClockSource::pll, 144'000'000, 8'000'000>;
using SpiHsiClock = brio::Clock<brio::ClockSource::pll, 144'000'000>;
static_assert(SpiPeerClock::hz == SpiHsiClock::hz && SpiPeerClock::pclk1_hz == SpiHsiClock::pclk1_hz &&
                  SpiPeerClock::pclk2_hz == SpiHsiClock::pclk2_hz,
              "the fallback must run every divisor at the same rate");
static_assert(SpiPeerClock::pclk1_hz == 72'000'000u,
              "a client follows a host at /2 of 72 MHz only on a 72 MHz bus of its own");

inline bool start_spi_clock(bool& crystal) {
    crystal = SpiPeerClock::init();
    return crystal || SpiHsiClock::init();
}

/// THE SERIAL PEER'S TIME BASE is the same 144 MHz: USART3 and UART4
/// count PCLK1, and 18.3's divisor floor of 16 on a 72 MHz bus is the 4.5
/// Mbaud both desk ports reach - the far port's top, so the peer's bus
/// must be as fast; and a PB1 timer then counts 144 MHz, SIX core cycles
/// a protocol tick, so every edge the waveform engine places and every
/// span the receiver stamps converts to ticks exactly.
using UsartPeerClock = SpiPeerClock;
static_assert(UsartPeerClock::pclk1_hz == 72'000'000u && UsartPeerClock::pclk1_hz / 16u == 4'500'000u,
              "18.3: a divisor of 16 on a 72 MHz PB1 is 4.5 Mbaud");
static_assert(UsartPeerClock::timclk1_hz == UsartPeerClock::hz && UsartPeerClock::hz % tick_hz == 0u,
              "a PB1 timer counts HCLK, a whole number of cycles a protocol tick");
inline constexpr uint32_t usart_cycles_per_tick = UsartPeerClock::hz / tick_hz;

inline bool start_usart_clock(bool& crystal) { return start_spi_clock(crystal); }

/// The part number after its CH32 prefix, NUL padded (the header).
inline void label(char out[8]) {
    static_assert(brio::device::part_name[0] == 'C' && brio::device::part_name[1] == 'H' &&
                      brio::device::part_name[2] == '3' && brio::device::part_name[3] == '2',
                  "the ident label is the part number after its CH32 prefix");
    const char* name = brio::device::part_name + 4;
    uint8_t i = 0;
    for (; i < 8u && name[i] != '\0'; ++i) {
        out[i] = name[i];
    }
    for (; i < 8u; ++i) {
        out[i] = '\0';
    }
}

/// The die, for a banner: the chip identifier and the two programmed
/// words of the UID (the third reads erased, docs/ch32vx03/nvm.md).
template <typename Out>
void print_die(const Out& out) {
    const brio::DeviceUid uid = brio::DeviceUid::read();
    brio::print(out, "CHIPID ", brio::hex(brio::chip_id()), " UID ", brio::hex(uid.word[0]), "-",
                brio::hex(uid.word[1]));
}

}  // namespace vx03peer
