// boards/da48/board.hpp - the self-built REV 1.2 48-pin PCB fitted with
// an AVR128DA48, board type `da48` (docs/boards/avr128da48.md): what is
// fitted on it, as constants a program reads at compile time. The same
// PCB fitted with an AVR64EA48 is another assembly and another board type,
// `ea48`, which is why nothing here can be read off the chip's header: an
// AVR128DA48 says nothing about the oscillator soldered next to it. The
// avrxt build puts this directory on the include path of every app it
// configures for `da48`, and an app writes `#include "board.hpp"`; what a
// board file is and what goes in it is docs/boards/README.md's, "The board
// file".
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
inline constexpr std::string_view type = "da48";

/// Whether this is the board of type `t`: decided at compile time at every
/// optimization level (consteval), so a choice keyed on the board costs
/// no code even where the optimizer folds nothing.
consteval bool is(std::string_view t) { return t == type; }

static_assert(brio::avr_die == brio::AvrDie::avr128da && brio::package_pins == 48,
              "boards/da48: the board carries an AVR128DA48 - the build names another part");

/// The static main clock of the board's HF source. The DA has no crystal
/// oscillator, so the board makes the clock itself - a 24 MHz quartz and
/// two inverters - and feeds it to PA0 as EXTCLK ("EXT CLK OPTS" in the
/// oscillator position); PA1 is then a GPIO. The OSCHF rate behind it,
/// the fallback, is the Clock type's default: the same 24 MHz.
using Clock = brio::Clock<brio::ClockSource::external, 24'000'000>;

/// A 32.768 kHz crystal on PF0/PF1.
inline constexpr bool xosc32k = true;

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

/// THE ANALOG NET: PD1, PD6 (the DAC's output) and PD7 (VREFA) wired
/// together, one net for the DAC-to-ADC and DAC-to-AC loops; a program
/// that drives one of the three drives the net, and a leg that needs one
/// of them as a free digital pad moves elsewhere. The net carries VREFA's
/// decoupling capacitor, 100 nF from PD7 to ground on the REV 1.2 PCB: a
/// pad driving the net charges it, and a DAC step moves it at the
/// buffer's current. The AVR64EA48 board's net is the same.
inline constexpr bool analog_net = true;

/// The resistor from the analog net to ground: the DAC sources it, and a
/// falling step is the resistor's rather than the buffer's weak sink.
inline constexpr uint16_t dac_load_ohms = 10'000;

/// The console: USART2 on its ALT1 route, PF4 (TX) / PF5 (RX), to the
/// PCB's isolated CH340B behind its "to UART" solder jumpers.
inline constexpr uint8_t console_usart = 2;
inline constexpr brio::UsartRoute console_route = brio::UsartRoute::alt1;

}  // namespace board
