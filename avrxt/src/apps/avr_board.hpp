// avr_board.hpp - the facts of the BOARD an avrxt app is built for, in
// one table: the first draft of the board file a portable application
// will name (CLAUDE.md, "The first portable example application"), kept
// beside the apps and out of brio/ until then.
//
// A board type names an ASSEMBLY: a PCB revision with the chip, the
// crystals and the jumpers fitted on it. Two of the three types below are
// the SAME PCB (REV 1.2) fitted differently, so nothing here can be read
// off the chip's header - an AVR128DA48 says nothing about the oscillator
// soldered next to it. The build passes the configure's board type as
// BRIO_BOARD_<type> (avrxt/CMakeLists.txt, avr_add_app()); a type this
// table does not know is a compile error.
//
// What the suites read:
//   BoardClock        the static main clock of the board's HF source:
//                     source, rate, and the OSCHF rate behind it (the
//                     fallback; on the AVR EA the OSCCFG fuse's rate, a
//                     provisioning fact of this board's chip)
//   board_xosc32k     a 32.768 kHz crystal on PF0/PF1
//   board_pa7_free    nothing on PA7 (the REV 1.2 PCB puts 100 nF there:
//                     an edge on it is an RC's, a clock on it a pad held
//                     at its short-circuit current)
//   board_clkout      CLK_PER may be put on PA7 (CLKOUT) - PA7 free
//   board_wave_crystal_pins
//                     a crystal pin whose oscillator did not start may be
//                     driven (pin_check's way of showing it); false where
//                     the crystals' pins are never driven, started or not
//   board_pc_pe_jumpers
//                     PC4-PE0, PC5-PE1, PC6-PE2, PC7-PE3 wired: each pair
//                     ONE net (and PE0..PE3 run on to the partner board),
//                     so no program drives both pins of a pair
//   board_console_*   the console's USART instance and route
//
// The packages without a board (db28, db32) have no row: their presets
// exist to build the family's link guard, and the one app they build
// (family_probe) reads no board fact.

#pragma once

#include <stdint.h>

#include "avrxt/clock.hpp"
#include "avrxt/usart.hpp"

#if defined(BRIO_BOARD_db48)
// The AVR128DB48 board: a 24 MHz crystal on PA0/PA1 (XOSCHF), no 32 kHz
// crystal, PA7 free (docs/boards/avr128db48.md).
using BoardClock = brio::Clock<brio::ClockSource::crystal, 24'000'000>;
inline constexpr bool board_xosc32k = false;
inline constexpr bool board_pa7_free = true;
inline constexpr bool board_wave_crystal_pins = true;
inline constexpr bool board_pc_pe_jumpers = false;
#elif defined(BRIO_BOARD_da48)
// The REV 1.2 PCB with an AVR128DA48: a 24 MHz oscillator (a quartz and
// two inverters) into PA0 as EXTCLK, 32.768 kHz on PF0/PF1, 100 nF on PA7
// (docs/boards/avr128da48.md).
using BoardClock = brio::Clock<brio::ClockSource::external, 24'000'000>;
inline constexpr bool board_xosc32k = true;
inline constexpr bool board_pa7_free = false;
inline constexpr bool board_wave_crystal_pins = true;
inline constexpr bool board_pc_pe_jumpers = false;
#elif defined(BRIO_BOARD_ea48)
// The REV 1.2 PCB with an AVR64EA48: a 20 MHz quartz on PA0/PA1
// (XOSCHF), 32.768 kHz on PF0/PF1, 100 nF on PA7, 5 V - the supply the
// 20 MHz needs (DS40002443A 35.10.5); the chip's OSCCFG left at the
// factory 0x00, OSCHF at 20 MHz; USART1 ALT1's PC4..PC7 jumpered to
// PE0..PE3 (docs/boards/avr64ea48.md). Its crystals' pins are never
// driven, started or not.
using BoardClock = brio::Clock<brio::ClockSource::crystal, 20'000'000, brio::ClockDiv::div1,
                               20'000'000>;
inline constexpr bool board_xosc32k = true;
inline constexpr bool board_pa7_free = false;
inline constexpr bool board_wave_crystal_pins = false;
inline constexpr bool board_pc_pe_jumpers = true;
#else
#error "avr_board.hpp: a board type this table does not know - add its row (an assembly: PCB, chip, crystals, jumpers)"
#endif

inline constexpr bool board_clkout = board_pa7_free;

/// The console: USART2 on its ALT1 route, PF4 (TX) / PF5 (RX), on every
/// board above (the DB board's adapter, the REV 1.2 PCB's isolated CH340B
/// behind its "to UART" jumpers).
inline constexpr uint8_t board_console_usart = 2;
inline constexpr brio::UsartRoute board_console_route = brio::UsartRoute::alt1;
