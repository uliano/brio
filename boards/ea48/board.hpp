// boards/ea48/board.hpp - the self-built REV 1.2 48-pin PCB fitted with
// an AVR64EA48, board type `ea48` (docs/boards/avr64ea48.md): what is
// fitted on it, as constants a program reads at compile time. The same
// PCB fitted with an AVR128DA48 is another assembly and another board
// type, `da48`. The avrxt build puts this directory on the include path
// of every app it configures for `ea48`, and an app writes
// `#include "board.hpp"`; what a board file is and what goes in it is
// docs/boards/README.md's, "The board file".
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
inline constexpr std::string_view type = "ea48";

/// Whether this is the board of type `t`: decided at compile time at every
/// optimization level (consteval), so a choice keyed on the board costs
/// no code even where the optimizer folds nothing.
consteval bool is(std::string_view t) { return t == type; }

static_assert(brio::avr_die == brio::AvrDie::avr64ea && brio::package_pins == 48,
              "boards/ea48: the board carries an AVR64EA48 - the build names another part");

/// The static main clock of the board's HF source: a 20 MHz quartz on
/// PA0/PA1 (XOSCHF, "EXT CLK OPTS" in the quartz position), at 5 V - the
/// supply the 20 MHz needs (DS40002443A 35.10.5). The last argument is
/// the OSCHF rate behind it, the fallback: on the AVR EA the OSCCFG fuse's
/// rate, which this board's chip keeps at the factory 0x00, 20 MHz.
using Clock = brio::Clock<brio::ClockSource::crystal, 20'000'000, brio::ClockDiv::div1, 20'000'000>;

/// A 32.768 kHz crystal on PF0/PF1.
inline constexpr bool xosc32k = true;

/// Nothing fitted on PA7: CLKOUT, a TCD's WOD on its DEFAULT route and
/// EVOUTA ALT1 may drive it and be read back as a level.
inline constexpr bool pa7_free = true;

/// CLK_PER may be put on PA7 (CLKOUT): PA7 is free.
inline constexpr bool clkout = pa7_free;

/// The crystals' pins are never driven on this board, started or not.
inline constexpr bool wave_crystal_pins = false;

/// THE PC-PE JUMPERS: PC4-PE0, PC5-PE1, PC6-PE2 and PC7-PE3 wired (USART1
/// ALT1's pads onto PORTE), each pair ONE net, and PE0..PE3 run on to the
/// partner board; so no program drives both pins of a pair.
inline constexpr bool pc_pe_jumpers = true;

/// THE ANALOG NET: PD1, PD6 (the DAC's output) and PD7 (VREFA) wired
/// together, one net for the DAC-to-ADC and DAC-to-AC loops; a program
/// that drives one of the three drives the net, and a leg that needs one
/// of them as a free digital pad moves elsewhere. The net carries VREFA's
/// decoupling capacitor, 100 nF from PD7 to ground on the REV 1.2 PCB: a
/// pad driving the net charges it, and a DAC step moves it at the
/// buffer's current.
inline constexpr bool analog_net = true;

/// The resistor from the analog net to ground: the DAC sources it, and a
/// falling step is the resistor's rather than the buffer's weak sink.
inline constexpr uint16_t dac_load_ohms = 10'000;

/// The console: USART2 on its ALT1 route, PF4 (TX) / PF5 (RX), to the
/// PCB's isolated CH340B behind its "to UART" solder jumpers.
inline constexpr uint8_t console_usart = 2;
inline constexpr brio::UsartRoute console_route = brio::UsartRoute::alt1;

}  // namespace board
