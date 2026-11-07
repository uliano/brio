// ch32_board.hpp - what the bench board around the chip decides for the
// apps of this project: the LED's pad, whether a crystal is fitted, the
// pads a suite may load with its pulls. App-level bench tooling; nothing
// in brio knows it exists. Constants and pin aliases only: no function
// here forms code.
//
// THE BOARD KEY IS THE PACKAGE. The project has no board files, and a
// board type is a part number (cli/bench/common.py's BOARD_TYPES), so a
// board fact is keyed on what the build names - the chip and its package
// (brio/ch32v00x/parts/) - which today is one board per key:
//   - the CH32V006's QFN32: the CH32V006K8U6 module - a 24 MHz crystal on
//     PA1/PA2, the LED jumpered to PC0 (docs/boards/ch32v006k8.md);
//   - the CH32V006's 20-pin package: WeAct's CH32V00x core board with the
//     CH32V006F8U6 - NO crystal (PA1 and PA2 go to the header alone), the
//     blue LED on PC4, active high through 10 kOhm to ground; PC0 is the
//     pair's SPI select (ch32_pair.hpp), never a LED;
//   - the CH32V003's TSSOP20: the CH32V003F4P6 board - a 24 MHz crystal
//     on PA1/PA2, its LEDs jumpered (docs/boards/ch32v003f4.md); the apps
//     drive PC0.
// A second board on one key (a CH32V003F4U6 board beside the F4P6 one:
// one 20-pin key) would need a key of its own.

#pragma once

#include <stdint.h>

#include <type_traits>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pin.hpp"

namespace ch32board {

/// WeAct's core board: the CH32V006 in its 20-pin package.
inline constexpr bool weact =
    brio::device::part == brio::Ch32Part::v006 && brio::device::package == brio::Ch32Package::pins20;

/// The LED every app drives: PC4 on the WeAct board, PC0 on the other two.
using Led = std::conditional_t<weact, brio::Pin<'C', 4>, brio::Pin<'C', 0>>;
/// Whether a high pad lights it: WeAct's LED hangs from its pad to ground;
/// a jumpered LED of the other two boards hangs from 3.3 V (the console's
/// reading of those boards, kept as it stands).
inline constexpr bool led_active_high = weact;

/// Whether a 24 MHz crystal sits on PA1/PA2. Without one, an app that
/// would start it runs the HSI's PLL instead, and a letter that judges
/// the crystal is not registered.
inline constexpr bool has_crystal = !weact;

/// THE LED LOADS ITS PAD: on the WeAct board PC4 carries 10 kOhm and a
/// blue LED to ground, which pulls a pad held up by its internal pull
/// (tens of kOhm) well below the rail. A letter that reads a pad through
/// its own pull-up takes this pad on the WeAct board instead of PC4: PA2,
/// ADC channel 0, free on that board (no crystal, no wire; the pin suite's
/// jumper, when fitted, joins it to PA1, a floating input meanwhile).
using PulledPad = std::conditional_t<weact, brio::Pin<'A', 2>, brio::Pin<'C', 4>>;

/// THE SOFTWARE-USB CIRCUIT, populated on the WeAct board (its schematic,
/// bottom left; the resistors stay fitted): PD3 through 33 Ohm (SB2) to the
/// USB-C's D+, PD4 through 33 Ohm (SB3) to D-, PD5 through 1.5 kOhm (SB4)
/// to D-. So PD4 and PD5 are joined through about 1.5 kOhm (measured with
/// a meter on two boards): PD4 IS NOT A FREE PAD there - its level follows
/// PD5, the console's transmit pad, and its pulls cannot hold it - and PD3
/// carries a 33 Ohm stub to the connector, which is left unplugged. A
/// letter that drives PD4, reads its level or its pull is not registered
/// on this board.
inline constexpr bool pd4_free = !weact;

/// THE PIN SUITE'S JUMPER, its driving and its sensing pad: PD2 to PD4 on
/// the other boards; PA2 to PA1 on the WeAct board, two pads nothing else
/// on that board uses (no crystal on PA1/PA2, no pair wire, not the LED,
/// not the USB pads). On the WeAct board the timer and serial suites' jumper
/// letters have no free pad to move to (their test files say why).
using JumperDrive = std::conditional_t<weact, brio::Pin<'A', 2>, brio::Pin<'D', 2>>;
using JumperSense = std::conditional_t<weact, brio::Pin<'A', 1>, brio::Pin<'D', 4>>;

}  // namespace ch32board
