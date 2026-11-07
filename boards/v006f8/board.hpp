// boards/v006f8/board.hpp - WeAct's CH32V00x core board fitted with a
// CH32V006F8U6, board type `v006f8`: what is fitted on it, as constants a
// program reads at compile time. The ch32v00x build puts this directory
// on the include path of every app it configures for `v006f8`, and an app
// writes `#include "board.hpp"`; what a board file is and what goes in it
// is docs/boards/README.md's, "The board file".
//
// Constants and pin aliases only: nothing here forms code, and nothing
// under brio/ includes it.

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pin.hpp"

#include <string_view>

namespace board {

/// The board type this file describes: the key the build and the bench
/// tool know the assembly by.
inline constexpr std::string_view type = "v006f8";

/// Whether this is the board of type `t`: decided at compile time at every
/// optimization level (consteval), so a choice keyed on the board costs
/// no code even where the optimizer folds nothing.
consteval bool is(std::string_view t) { return t == type; }

static_assert(brio::device::part == brio::Ch32Part::v006 && brio::device::package == brio::Ch32Package::pins20,
              "boards/v006f8: the board carries a CH32V006 in its 20-pin package - the build names another part");

/// The LED every app drives: the board's blue LED on PC4, through 10 kOhm
/// to ground. PC0 is the pair's SPI select (ch32v00x/src/apps/ch32_pair.hpp),
/// never a LED.
using Led = brio::Pin<'C', 4>;
/// Whether a high pad lights it: the LED hangs from its pad to ground.
inline constexpr bool led_active_high = true;

/// NO crystal: PA1 and PA2 go to the header alone. An app that would
/// start one runs the HSI's PLL instead, and a letter that judges the
/// crystal is not registered.
inline constexpr bool has_crystal = false;

/// PC4 carries the LED: a letter that wants a pad with no wire takes
/// another.
inline constexpr bool pc4_free = false;

/// THE LED LOADS ITS PAD: PC4's 10 kOhm and blue LED to ground pull a
/// pad held up by its internal pull (tens of kOhm) well below the rail.
/// A letter that reads a pad through its own pull-up takes PA2 instead,
/// ADC channel 0, free on this board (no crystal, no wire; the pin suite's
/// jumper, when fitted, joins it to PA1, a floating input meanwhile).
using PulledPad = brio::Pin<'A', 2>;

/// THE SOFTWARE-USB CIRCUIT, populated (the board's schematic, bottom
/// left; the resistors stay fitted): PD3 through 33 Ohm (SB2) to the
/// USB-C's D+, PD4 through 33 Ohm (SB3) to D-, PD5 through 1.5 kOhm (SB4)
/// to D-. So PD4 and PD5 are joined through about 1.5 kOhm (measured with
/// a meter on two boards): PD4 IS NOT A FREE PAD here - its level follows
/// PD5, the console's transmit pad, and its pulls cannot hold it - and PD3
/// carries a 33 Ohm stub to the connector, which is left unplugged. A
/// letter that drives PD4, reads its level or its pull is not registered
/// on this board.
inline constexpr bool pd4_free = false;

/// THE PIN SUITE'S JUMPER, its driving and its sensing pad: PA2 to PA1,
/// two pads nothing else on this board uses (no crystal on PA1/PA2, no
/// pair wire, not the LED, not the USB pads). The timer and serial
/// suites' jumper letters have no free pad to move to on this board
/// (their test files say why).
using JumperDrive = brio::Pin<'A', 2>;
using JumperSense = brio::Pin<'A', 1>;

}  // namespace board
