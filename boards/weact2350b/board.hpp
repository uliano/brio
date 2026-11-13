// boards/weact2350b/board.hpp - WeAct Studio's RP2350B core board, board
// types `weact2350b` and `weact2350b-rv` (one piece of hardware, built for
// the Cortex-M33 pair and for the Hazard3 pair; docs/boards/weact-rp2350b.md):
// what is fitted on it, as constants a program reads at compile time. The
// rp2350 build puts this directory on the include path of every app it
// configures for the QFN-80 package, and an app writes
// `#include "board.hpp"`; what a board file is and what goes in it is
// docs/boards/README.md's, "The board file".
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
inline constexpr std::string_view type = "weact2350b";

/// Whether this is the board `t`: decided at compile time at every
/// optimization level (consteval), so a choice keyed on the board costs
/// no code even where the optimizer folds nothing.
consteval bool is(std::string_view t) { return t == type; }

static_assert(brio::package_known && brio::package == brio::Package::qfn80,
              "boards/weact2350b: the board carries an RP2350 in the QFN-80 package - the build names another");

/// The user LED on GP25, lit by a high pad (the board's page).
using Led = brio::Pin<25>;
inline constexpr bool led_active_high = true;

/// A user KEY beside BOOT and RESET: a button to ground on GP23 (the
/// board's page). A driven output there would meet the button.
inline constexpr bool has_key = true;
inline constexpr bool gp23_free = false;

/// GP24 is on the header rows with every other user GPIO, and the
/// board's page names no circuit on it.
inline constexpr bool gp24_free = true;

/// GP29 is on the header rows and the page names no circuit on it, but it
/// DOES NOT DRIVE on the board at the bench: its output reads 1 while
/// driven low at 12 mA, with or without a wire on it (the board's page,
/// measured).
/// A wave on it harms nothing; no letter relies on driving it.
inline constexpr bool gp29_free = true;

// The footprint for a second flash or a PSRAM, whose chip select would be
// GP0, is empty (the board's page): GP0 is free and carries the console's
// transmit line.

}  // namespace board
