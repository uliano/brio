// boards/db48/board.hpp - the self-built AVR128DB48 board, board type
// `db48` (docs/boards/avr128db48.md): what is fitted on it, as constants
// a program reads at compile time. The avrxt build puts this directory on
// the include path of every app it configures for `db48`, and an app
// writes `#include "board.hpp"`; what a board file is and what goes in
// it is docs/boards/README.md's, "The board file".
//
// Constants and type aliases only: nothing here forms code, and nothing
// under brio/ includes it.

#pragma once

#include <stdint.h>

#include "avrxt/clock.hpp"
#include "avrxt/device_tables.hpp"
#include "avrxt/usart.hpp"

#include <string_view>

namespace board {

/// The board type this file describes: the key the build and the bench
/// tool know the assembly by.
inline constexpr std::string_view type = "db48";

/// Whether this is the board of type `t`: decided at compile time at every
/// optimization level (consteval), so a choice keyed on the board costs
/// no code even where the optimizer folds nothing.
consteval bool is(std::string_view t) { return t == type; }

static_assert(brio::avr_die == brio::AvrDie::avr128db && brio::package_pins == 48,
              "boards/db48: the board carries an AVR128DB48 - the build names another part");

/// The static main clock of the board's HF source: the 24 MHz crystal on
/// PA0/PA1 (XOSCHF), so those two pins are not GPIOs. The OSCHF rate
/// behind it, the fallback, is the Clock type's default: the same 24 MHz.
using Clock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;

/// No 32.768 kHz crystal on PF0/PF1: the RTC and the kernel tick run on
/// the internal OSC32K, and PF0/PF1 are free.
inline constexpr bool xosc32k = false;

/// Nothing fitted on PA7: CLKOUT, a TCD's WOD on its DEFAULT route and
/// EVOUTA ALT1 may drive it and be read back as a level.
inline constexpr bool pa7_free = true;

/// CLK_PER may be put on PA7 (CLKOUT): PA7 is free.
inline constexpr bool clkout = pa7_free;

/// A crystal pin whose oscillator did not start may be driven (the
/// pin-check tool's way of showing it).
inline constexpr bool wave_crystal_pins = true;

/// No PC-PE jumpers: PC4..PC7 and PE0..PE3 are separate pads.
inline constexpr bool pc_pe_jumpers = false;

/// No analog net: PD1, PD6 and PD7 are separate pads.
inline constexpr bool analog_net = false;

/// No resistor from an analog net to ground (0 = none).
inline constexpr uint16_t dac_load_ohms = 0;

/// The console: USART2 on its ALT1 route, PF4 (TX) / PF5 (RX), to the
/// board's CH340 bridge.
inline constexpr uint8_t console_usart = 2;
inline constexpr brio::UsartRoute console_route = brio::UsartRoute::alt1;

}  // namespace board
