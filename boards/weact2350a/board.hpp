// boards/weact2350a/board.hpp - WeAct Studio's RP2350A core board V2.1,
// board types `weact2350a` and `weact2350a-rv` (one piece of hardware,
// built for the Cortex-M33 pair and for the Hazard3 pair;
// docs/boards/weact-rp2350a.md): what is fitted on it, as constants a
// program reads at compile time. The rp2350 build puts this directory on
// the include path of every app it configures for the QFN-60 package, and
// an app writes `#include "board.hpp"`; what a board file is and what goes
// in it is docs/boards/README.md's, "The board file".
//
// THE SOURCE. WeAct publishes the schematics of the V10 and the V20 of
// this board (the WeActStudio.RP2350ACoreBoard repository on GitHub), not
// of the V2.1 at the bench. What the V2.1 shows by eye - the LED, the two
// buttons, the header rows - is stated as the board's; what only the
// schematic shows (the circuits on GP23, GP24 and GP29) is stated as the
// V20's, unmeasured on the V2.1.
//
// Constants and pin aliases only: nothing here forms code, and nothing
// under brio/ includes it.

#pragma once

#include <stdint.h>

#include "rp2350/device.hpp"
#include "rp2350/pin.hpp"

#include <string_view>

namespace board {

/// The board this file describes: the key the bench tool and the pair
/// plans know the assembly by (the -rv board type is the same board).
inline constexpr std::string_view type = "weact2350a";

/// Whether this is the board `t`: decided at compile time at every
/// optimization level (consteval), so a choice keyed on the board costs
/// no code even where the optimizer folds nothing.
consteval bool is(std::string_view t) { return t == type; }

static_assert(brio::package_known && brio::package == brio::Package::qfn60,
              "boards/weact2350a: the board carries an RP2350 in the QFN-60 package - the build names another");

/// The user LED on GP25, lit by a high pad: the V20 schematic's green LED
/// through 5.1 kOhm (R7) to ground.
using Led = brio::Pin<25>;
inline constexpr bool led_active_high = true;

/// NO user key: the board has two buttons, BOOT (the flash's select, the
/// bootrom's loader when held at reset) and RESET (the RUN pin), and
/// neither reaches a GPIO.
inline constexpr bool has_key = false;

/// GP23 IS NOT FREE: on the V20 it is the MODE pin of the switching
/// regulator that makes the 3V3 rail (U2), pulled to ground by 100 kOhm
/// (R20) - low the regulator's power-saving mode, high its fixed-frequency
/// mode. It is not on the header rows.
inline constexpr bool gp23_free = false;

/// GP24 IS NOT FREE: on the V20 it senses VBUS through a divider of two
/// 100 kOhm resistors (R28 from VBUS, R29 to ground), so it reads half
/// the USB supply. It is not on the header rows.
inline constexpr bool gp24_free = false;

/// GP29 IS NOT FREE: on the V20 it senses VSYS - a divider of two 100 kOhm
/// resistors (R23, R24, with 1 nF across the lower), passed through the
/// MOSFET Q1 whose gate is the 3V3 rail, with 100 kOhm (R25) from the pad
/// to ground - and it is ADC input 3. It is not on the header rows.
inline constexpr bool gp29_free = false;

// The V20 has a footprint joining GP0 to the flash's select (R14, marked
// not fitted): nothing is on it, so GP0 is free and carries the console's
// transmit line.

}  // namespace board
